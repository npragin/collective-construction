

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Pose, PoseStamped
from cv_bridge import CvBridge
import cv2
# from retriever_robots.utils import create_rotation_matrix

# from retriever_msgs.msg import PoseStatus

from block_interfaces.msg import BlockPose
from cc_interfaces.msg import Block

import message_filters

from scipy.spatial.transform import Rotation

from tf2_ros import Buffer, TransformListener

import tf2_geometry_msgs

from visualization_msgs.msg import Marker, MarkerArray

from collections import deque

"""
This code was developed by Zane and Atharv and edited slightly by Liam
"""

# MARKER_SIZE = 0.043
# MARKER_SIZE = 0.048
MARKER_SIZE = 0.055
OBJ_PTS = np.array(
    [
        [-MARKER_SIZE / 2, MARKER_SIZE / 2, 0.0],
        [MARKER_SIZE / 2, MARKER_SIZE / 2, 0.0],
        [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0.0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0.0],
    ],
    dtype=np.float64,
)


class DetectBlock(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        self.get_logger().info('DetectBlock Node is Up!')

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ignore blocks more then this distance away. 
        self.max_block_dist = 2.0

        self.marker_pub = self.create_publisher(MarkerArray, 'found_blocks', 10)
        self.found_blocks = []

        self.bridge = CvBridge() # converts between ros2 image messages and openCV images w

        # publishes the visible block poses to the central planner
        self.vis_pub = self.create_publisher(
            # BlockPose, f"{self.get_namespace()}/scout_report", 10
            Block, f"{self.get_namespace()}/scout_report", 10
        )

        # describe how a real camera converts 3D poitns into image pizels
        self.camera_matrix = None
        self.distortion_coeffs = None

        # self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_25h9)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # gets actual rgb image
        self.color_sub = message_filters.Subscriber(
            self, Image, f"{self.get_namespace()}/camera/color/image_raw"
        )
        # camera calibration params
        self.color_info = message_filters.Subscriber(
            self,
            CameraInfo,
            f"{self.get_namespace()}/camera/color/camera_info",
        )

        queue_size = 1
        slop = 0.2

        # waits until it finds messages from multiple topics within a close enough time stamp, then passes it through the callback on next line
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.color_info], queue_size, slop
        )

        self.detection_queue = deque(maxlen=20)
        self.create_timer(0.05, self.process_detection_queue)  # drain queue every 50ms

        # callback that happens when synced messages come from above
        self.ts.registerCallback(self.cam_callback)

        self.logger = self.get_logger()
        self.logger.info(f"Launched Block Detection Node for {self.get_namespace()}")

    def cam_callback(self, img_msg, cam_info_msg):
        self.camera_info_callback(cam_info_msg)
        self.image_callback(img_msg)

    
    def camera_info_callback(self, msg):
        if self.camera_matrix is not None and self.distortion_coeffs is not None:
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        
        # TODO swap this with map possible map -> base_link
        # pose_status = PoseStatus()
        # pose_status.tag_in_frame = False
        try:
            if self.camera_matrix is None or self.distortion_coeffs is None:
                self.logger.warn("Camera info not received yet.")
                return

            # converts ros message of image to cv image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # inputs: (image, dict) -> outputs: (list of corner coords)
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.parameters
            )

            # self.get_logger().info(f'ids: {ids}')
            # self.get_logger().info(f'corners: {corners}')

            # if there exists atleast one marker
            if ids is not None:
                
                self.logger.debug(f"Found {len(ids)} tags: {ids.flatten()}")
                for i in range(len(ids)):
                        
                    candidate_block_id = ids[i].item()
                    corner = corners[i][0]
                    self.get_logger().info(f'id: {candidate_block_id}')
    
                    # input (3D points like tag 4 corners , 2D corre. projections in image)
                    # output (rot vector and translation of the camera)
    
                    # both of these are w.r.t to the camera.
                    # rvec: rotation vector. ex: [0, 0, 1.5] -> rotate around z axis by 1.5 radians
                    # tvec: translation vector. ex: [1,2,3] -> object  is 1m on x, 2m on y, and 3 on z
                    ok, rvec, tvec = cv2.solvePnP(
                        OBJ_PTS,
                        # corners[0][0],
                        corner,
                        self.camera_matrix,
                        self.distortion_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    # self.get_logger().info(f'rvec: {rvec}\ntvec: {tvec}')
    
                    # if PnP can be sovled for tag 
                    if ok:
    
                        # converts rotation vector into 3x3 rot matrix
                        R_marker_to_cam, _ = cv2.Rodrigues(rvec)
                        # Image X is robot -Y, Image Y is robot -Z, Image Z is robot X
                        R_image_to_robot_axes = np.array(
                            [
                                [0, 0, 1],
                                [-1, 0, 0],
                                [0, -1, 0],
                            ]
                        )

                        R_cam_to_robot = R_image_to_robot_axes
    
    
                        R_marker_to_robot = R_cam_to_robot @ R_marker_to_cam
                        R_marker_to_robot = R_marker_to_robot
    
                        T_cam_to_robot = np.array(
                            [[-0.1], [0], [0]]
                        )  # camera is 10cm in front of the robot axis
                        # self.get_logger().info(f'tvec: {tvec}')
    
                        T_marker_in_cam = tvec.reshape(3, 1)
                        T_marker_to_robot = (
                            R_cam_to_robot @ T_marker_in_cam + T_cam_to_robot
                        )

                        self.get_logger().info(f'T_marker_to_robot: {T_marker_to_robot}')
                        dist_to_block = np.hypot(T_marker_to_robot[0].item(), T_marker_to_robot[1].item())
                        self.get_logger().info(f'distance to block: {dist_to_block}')
                        if dist_to_block > self.max_block_dist:
                            continue
    
                        self.logger.debug(
                            f"Tag Detected: Marker center is {T_marker_to_robot[0]} m away,  {T_marker_to_robot[1]} m to the left, and {T_marker_to_robot[2]} m down)",
                            throttle_duration_sec=1.0,
                        )
                        
                        candidate_pose_stamped = PoseStamped()
                        # candidate_pose_stamped.header.frame_id = f'{self.get_namespace()}/aruco_31'[1:]
                        candidate_pose_stamped.header.frame_id = 'aruco_31'
                        # candidate_pose_stamped.header.frame_id = 'sierra/base_link'
                        candidate_pose_stamped.header.stamp = msg.header.stamp

                        candidate_pose_stamped.pose.position.x = float(T_marker_to_robot[0])
                        candidate_pose_stamped.pose.position.y = float(T_marker_to_robot[1])
                        candidate_pose_stamped.pose.position.z = float(T_marker_to_robot[2])
                        
                        x, y, z, w = Rotation.from_matrix(R_marker_to_robot).as_quat()
                        candidate_pose_stamped.pose.orientation.x = x
                        candidate_pose_stamped.pose.orientation.y = y
                        candidate_pose_stamped.pose.orientation.z = z
                        candidate_pose_stamped.pose.orientation.w = w
    
                        self.get_logger().info(f'Pose in frame_id baselink: \
                            x: {candidate_pose_stamped.pose.position.x}, \
                            y: {candidate_pose_stamped.pose.position.y}, \
                            z: {candidate_pose_stamped.pose.position.z}')
    

                        # replace the tf_buffer.transform() call with this
                        self.detection_queue.append({
                            'stamp': msg.header.stamp,
                            'pose': candidate_pose_stamped,
                            'block_id': candidate_block_id,
                        })

                        continue  # skip the rest, let the timer handle it d
                        

                else:
                    self.logger.debug(
                        "Could not solve PnP for detected tag.",
                        throttle_duration_sec=1.0,
                    )

            else:
                self.logger.debug(
                    "No tags detected in the image.", throttle_duration_sec=1.0
                )
                
            # publish markes of blocks for rviz
            self.publish_markers()


        except Exception as e:
            self.logger.error(f"Error converting image: {e}")


    def process_detection_queue(self):
        still_pending = deque()
        for detection in self.detection_queue:
            stamp = detection['stamp']
            pose = detection['pose']
            block_id = detection['block_id']
    
            # if self.tf_buffer.can_transform('world', 'sierra/base_link', stamp):
            if self.tf_buffer.can_transform('world', 'aruco_31', stamp):

                try:
                    transformed_pose = self.tf_buffer.transform(pose, 'world')
    
                    candidate_block = Block()
                    candidate_block.type = Block.TYPE_B
                    candidate_block.pose = transformed_pose
    
                    duplicate = False
                    for found_block, found_id in self.found_blocks:
                        if found_id == block_id:
                            found_block.pose = transformed_pose
                            duplicate = True
    
                    if not duplicate:
                        self.found_blocks.append((candidate_block, block_id))
                        self.vis_pub.publish(candidate_block)
                        self.publish_markers()
    
                except Exception as e:
                    self.get_logger().warn(f'Transform failed even after can_transform: {e}')
            else:
                still_pending.append(detection)  # TF not here yet, try again next timer tick
    
        self.detection_queue = still_pending


    def publish_markers(self):
        marker_array = MarkerArray()
        for block, block_id in self.found_blocks:
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'found_blocks'
            marker.id = block_id
            marker.type = Marker.CUBE
            # this is how we remove old markers
            marker.action = Marker.ADD 
            marker.pose = block.pose.pose
    
            marker.scale.x = 0.1
            marker.scale.y = 0.2
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DetectBlock("detect_block")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()