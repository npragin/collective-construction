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

import message_filters

from scipy.spatial.transform import Rotation

from tf2_ros import Buffer, TransformListener

import tf2_geometry_msgs

from visualization_msgs.msg import Marker, MarkerArray

"""
This code was developed by Zane and Atharv and edited slightly by Liam
"""

MARKER_SIZE = 0.043
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

        self.marker_pub = self.create_publisher(MarkerArray, 'found_blocks', 10)
        self.found_blocks = []

        self.bridge = CvBridge() # converts between ros2 image messages and openCV images

        # publishes the visible block poses to the central planner
        self.vis_pub = self.create_publisher(
            BlockPose, f"{self.get_namespace()}/visible_block_pose", 10
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

            self.get_logger().info(f'ids: {ids}')
            self.get_logger().info(f'corners: {corners}')

            # if there exists atleast one marker
            if ids is not None:
                
                self.logger.debug(f"Found {len(ids)} tags: {ids.flatten()}")
                for i in range(len(ids)):
                        
                    block_id = ids[i][0]
                    corner = corners[i][0]
                    self.get_logger().info(f'id: {block_id}')
    
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
                    self.get_logger().info(f'rvec: {rvec}\ntvec: {tvec}')
    
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
    
                        # this code accounts for the retriever camera pointing 30 degrees down.
                        # R_cam_angle_to_robot = create_rotation_matrix(
                        #     pitch=30, units="degrees"
                        # )
                        # R_cam_to_robot = R_cam_angle_to_robot @ R_image_to_robot_axes
                        R_cam_to_robot = R_image_to_robot_axes
    
    
                        R_marker_to_robot = R_cam_to_robot @ R_marker_to_cam
                        R_marker_to_robot = R_marker_to_robot
    
                        T_cam_to_robot = np.array(
                            [[-0.1], [0], [0]]
                        )  # camera is 10cm in front of the robot axis
    
                        T_marker_in_cam = tvec.reshape(3, 1)
                        T_marker_to_robot = (
                            R_cam_to_robot @ T_marker_in_cam + T_cam_to_robot
                        )
    
                        self.logger.debug(
                            f"Tag Detected: Marker center is {T_marker_to_robot[0]} m away,  {T_marker_to_robot[1]} m to the left, and {T_marker_to_robot[2]} m down)",
                            throttle_duration_sec=1.0,
                        )
    
                        candidate_pose_stamped = PoseStamped()
                        candidate_pose_stamped.header.frame_id = f'{self.get_namespace()}/base_link'[1:]
                        candidate_pose_stamped.pose.position.x = float(T_marker_to_robot[0])
                        candidate_pose_stamped.pose.position.y = float(T_marker_to_robot[1])
                        candidate_pose_stamped.pose.position.z = float(T_marker_to_robot[2])
                        
                        x, y, z, w = Rotation.from_matrix(R_marker_to_robot).as_quat()
                        candidate_pose_stamped.pose.orientation.x = x
                        candidate_pose_stamped.pose.orientation.y = y
                        candidate_pose_stamped.pose.orientation.z = z
                        candidate_pose_stamped.pose.orientation.w = w
    
                        self.get_logger().info(f'Pose in frame_id baselink: { \
                            candidate_pose_stamped.pose.position.x, \
                            candidate_pose_stamped.pose.position.y, \
                            candidate_pose_stamped.pose.position.z}')
    
                        
                        candidate_pose_stamped = self.tf_buffer.transform(
                            candidate_pose_stamped,
                            'map'
                        )
    
                        self.get_logger().info(f'Pose in frame_id world: { \
                            candidate_pose_stamped.pose.position.x, \
                            candidate_pose_stamped.pose.position.y, \
                            candidate_pose_stamped.pose.position.z}')
                    
    
                        candidate_block = BlockPose()
                        # candidate_block.id = block_id # TODO make this the actual ID
                        candidate_block.id = 0 # TODO make this the actual ID

                        candidate_block.pose_stamped = candidate_pose_stamped
    
                        # check to see if this is a duplicate block
                        for found_block in self.found_blocks:   
                            if found_block.id == candidate_block.id:
                                self.get_logger().info(f'Block {found_block.id} is a duplicate')
                                continue
                        
                        # this a new block
                        self.found_blocks.append(candidate_block)
                        
                        # publish the newly found block
                        self.vis_pub.publish(candidate_block) 
    
                        # publish markes of blocks for rviz
                        self.publish_markers()

                else:
                    self.logger.debug(
                        "Could not solve PnP for detected tag.",
                        throttle_duration_sec=1.0,
                    )

            else:
                self.logger.debug(
                    "No tags detected in the image.", throttle_duration_sec=1.0
                )

            # if not pose_status.tag_in_frame:
            #     pose_status.block_in_frame, x, y = self.segment_color(
            #         cv_image
            #     )  # if no tags are detected, try to segment based on color as a fallback
            #     if pose_status.block_in_frame:
            #         # create a fake pose with a y position scaled based on the negative x value of the image. Make a rough x pose based on y in frame
            #         pose_status.pose.position.x = 0.5 + max(
            #             min(-0.001 * (y - cv_image.shape[0] / 2), 0.5), -0.5
            #         )
            #         pose_status.pose.position.y = max(
            #             min(-0.001 * (x - cv_image.shape[1] / 2), 0.5), -0.5
            #         )
            #         pose_status.pose.position.z = 0.0
            #         pose_status.pose.orientation.w = 1.0
            #         self.logger.debug(
            #             f"Tag not detected, using color segmentation. Estimated pose: ({pose_status.pose.position.x}, {pose_status.pose.position.y}, {pose_status.pose.position.z})",
            #             throttle_duration_sec=1.0,
            #         )

            # self.vis_pub.publish(pose_status)

        except Exception as e:
            self.logger.error(f"Error converting image: {e}")

    def publish_markers(self):
        marker_array = MarkerArray()
        for block in self.found_blocks:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'found_blocks'
            marker.id = block.id
            marker.type = Marker.CUBE
            # this is how we remove old markers
            marker.action = Marker.ADD 
            marker.pose = block.pose_stamped.pose
    
            marker.scale.x = 0.4
            marker.scale.y = 0.8
            marker.scale.z = 0.4
            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    # def segment_color(self, cv_image):
    #         # Convert the image to HSV color space for better color segmentation
    #         hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #         # Define the lower and upper bounds for the block's color in HSV space
    #         # do orange instead
    #         lower_color = np.array([2, 100, 100])  # lower bound (orange color)
    #         upper_color = np.array([10, 255, 255])  # Example upper

    #         # Create a mask using the defined color bounds
    #         mask = cv2.inRange(hsv_image, lower_color, upper_color)

    #         # Find contours in the mask
    #         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #         if contours:
    #             largest_contour = max(contours, key=cv2.contourArea)
    #             M = cv2.moments(largest_contour)
    #             if M["m00"] > 50:
    #                 cX = int(M["m10"] / M["m00"])
    #                 cY = int(M["m01"] / M["m00"])
    #                 self.logger.debug(f"Segmented block at pixel coordinates: ({cX}, {cY})")
    #                 return True, cX, cY
    #         return False, None, None



def main(args=None):
    rclpy.init(args=args)
    node = DetectBlock("detect_block")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()