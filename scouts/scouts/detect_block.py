import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Pose
from cv_bridge import CvBridge
import cv2

# from retriever_robots.utils import create_rotation_matrix

# from retriever_msgs.msg import PoseStatus

from block_interfaces.msg import BlockPose

import message_filters

from scipy.spatial.transform import Rotation

from tf2_ros import Buffer, TransformListener


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

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.bridge = CvBridge() # converts between ros2 image messages and openCV images


        # self.pub = self.create_publisher(Twist, f"{self.get_namespace()}/cmd_vel", 10)

        # communicates the location of the identified block back to the retriever node. This is a custom topic, not a standard ROS topic, so we can change it as needed.
        self.vis_pub = self.create_publisher(
            Pose, f"{self.get_namespace()}/visible_block_pose", 10
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

            # if there exists atleast one marker
            if ids is not None:
                
                self.logger.debug(f"Found {len(ids)} tags: {ids.flatten()}")

                # input (3D points like tag 4 corners , 2D corre. projections in image)
                # output (rot vector and translation of the camera)

                # both of these are w.r.t to the camera.
                # rvec: rotation vector. ex: [0, 0, 1.5] -> rotate around z axis by 1.5 radians
                # tvec: translation vector. ex: [1,2,3] -> object  is 1m on x, 2m on y, and 3 on z
                ok, rvec, tvec = cv2.solvePnP(
                    OBJ_PTS,
                    corners[0][0],
                    self.camera_matrix,
                    self.distortion_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )

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

                    block_pose = BlockPose()
                    block_pose.pose_stamped.header.stamp = self.get_clock().now().to_msg()
                    block_pose.pose_stamped.header.frame_id = 'base_link'
                    block_pose.pose_stamped.pose.position.x = float(T_marker_to_robot[0])
                    block_pose.pose_stamped.pose.position.y = float(T_marker_to_robot[1])
                    block_pose.pose_stamped.pose.position.z = float(T_marker_to_robot[2])
                    
                    x, y, z, w = Rotation.from_matrix(R_marker_to_robot).as_quat()
                    block_pose.pose_stamped.pose.orientation.x = x
                    block_pose.pose_stamped.pose.orientation.y = y
                    block_pose.pose_stamped.pose.orientation.z = z
                    block_pose.pose_stamped.pose.orientation.w = w

                    self.get_logger().info(f'Pose in frame_id baselink: { block_pose.pose_stamped.pose.position.x, block_pose.pose_stamped.pose.position.y, block_pose.pose_stamped.pose.position.z}')

                    block_pose_world = self.tf_buffer.transform(
                        block_pose,
                        'map'
                    )

                    self.get_logger().info(f'Pose in frame_id world: { block_pose_world.pose_stamped.pose.position.x, block_pose_world.pose_stamped.pose.position.y, block_pose_world.pose_stamped.pose.position.z}')

                    # needs to send msg to central planner of block in global frame
                    # needs to publish marker of block in global frame

                    # step 1: convert block to global frame
                    # step 2: publish it on visible_block_pose
                    # step 3: publish a point market on rviz


                    # pose = Pose()

                    # pose.position.x = float(T_marker_to_robot[0])
                    # pose.position.y = float(T_marker_to_robot[1])
                    # pose.position.z = float(T_marker_to_robot[2])

                    # x, y, z, w = Rotation.from_matrix(R_marker_to_robot).as_quat()
                    # pose.orientation.x = x
                    # pose.orientation.y = y
                    # pose.orientation.z = z
                    # pose.orientation.w = w

                    # self.vis_pub.publish(pose)

                    # pose_status.tag_in_frame = True
                    # pose_status.pose = pose

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