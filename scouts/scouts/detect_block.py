import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Pose, PoseStamped
from cv_bridge import CvBridge
import cv2

from block_interfaces.msg import BlockPose
from cc_interfaces.msg import Block

import message_filters

from scipy.spatial.transform import Rotation

from tf2_ros import Buffer, TransformListener

import tf2_geometry_msgs

from visualization_msgs.msg import Marker, MarkerArray

from collections import deque

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

class TrackedBlock():
    """
    Tracks a block currently in view.
    Accumulates a running average of its world-frame position.
    """
    def __init__(self, block_id, time):
        self.block_id = block_id
        self.avg_x = 0.0
        self.avg_y = 0.0
        self.avg_z = 0.0
        self.avg_qx = 0.0
        self.avg_qy = 0.0
        self.avg_qz = 0.0
        self.avg_qw = 1.0
        self.n = 0  # number of samples accumulated
        self.time_seen = time

    def update(self, world_pose: PoseStamped):
        """Incremental moving average of position and orientation."""
        self.n += 1
        p = world_pose.pose.position
        o = world_pose.pose.orientation

        self.avg_x += (p.x - self.avg_x) / self.n
        self.avg_y += (p.y - self.avg_y) / self.n
        self.avg_z += (p.z - self.avg_z) / self.n

        self.avg_qx += (o.x - self.avg_qx) / self.n
        self.avg_qy += (o.y - self.avg_qy) / self.n
        self.avg_qz += (o.z - self.avg_qz) / self.n
        self.avg_qw += (o.w - self.avg_qw) / self.n

    def get_averaged_pose(self, time, frame_id='world') -> PoseStamped:
        """Return the current moving average as a PoseStamped."""
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = time
        ps.pose.position.x = self.avg_x
        ps.pose.position.y = self.avg_y
        ps.pose.position.z = self.avg_z

        # renormalize quaternion
        q = np.array([self.avg_qx, self.avg_qy, self.avg_qz, self.avg_qw])
        norm = np.linalg.norm(q)
        if norm > 1e-6:
            q /= norm
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps

class DetectBlock(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        self.id2type = {
            0: Block.TYPE_A,
            1: Block.TYPE_A,
            2: Block.TYPE_A,
            3: Block.TYPE_A,
            4: Block.TYPE_A,
            5: Block.TYPE_A,
            # 6: ,
            # 7: ,
            # 8: ,
            # 9: ,
            10: Block.TYPE_B,
            11: Block.TYPE_B,
            12: Block.TYPE_B,
            13: Block.TYPE_B,
            # 14: ,
            # 15: ,
            # 16: ,
            # 17: ,
            # 18: ,
            # 19: ,
            20: Block.TYPE_C,
            21: Block.TYPE_C,
            22: Block.TYPE_C,

        }

        self.get_logger().info('DetectBlock Node is Up!')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.max_block_dist = 2.0

        self.marker_pub = self.create_publisher(MarkerArray, 'found_blocks', 10)
        self.found_blocks = [] # every seen block, ever
        self.visible_block_ids = set() # block ids robot can visually see
        self.published_blocks = [] # blocks actually published f

        self.bridge = CvBridge()

        self.vis_pub = self.create_publisher(
            Block, f"{self.get_namespace()}/scout_report", 10
        )

        self.camera_matrix = None
        self.distortion_coeffs = None

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.color_sub = message_filters.Subscriber(
            self, Image, f"{self.get_namespace()}/camera/color/image_raw"
        )
        self.color_info = message_filters.Subscriber(
            self, CameraInfo, f"{self.get_namespace()}/camera/color/camera_info",
        )

        queue_size = 1
        slop = 0.2

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.color_info], queue_size, slop
        )

        self.detection_queue = deque(maxlen=20)
        self.create_timer(0.05, self.process_detection_queue)

        self.ts.registerCallback(self.cam_callback)

        self.logger = self.get_logger()
        self.logger.info(f"Launched Block Detection Node for {self.get_namespace()}")

        self.create_timer(0.5, self.publish_blocks)


    def cam_callback(self, img_msg, cam_info_msg):
        self.camera_info_callback(cam_info_msg)
        self.image_callback(img_msg)


    def camera_info_callback(self, msg):
        if self.camera_matrix is not None and self.distortion_coeffs is not None:
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)


    def image_callback(self, msg):
        try:
            if self.camera_matrix is None or self.distortion_coeffs is None:
                self.logger.warn("Camera info not received yet.")
                return
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.parameters
            )
            curr_visible_block_ids = set()

            if ids is not None:
                self.logger.debug(f"Found {len(ids)} tags: {ids.flatten()}")

                for i in range(len(ids)):
                    candidate_block_id = ids[i].item()
                    corner = corners[i][0]
                    # self.get_logger().info(f'id: {candidate_block_id}') g

                    ok, rvec, tvec = cv2.solvePnP(
                        OBJ_PTS,
                        corner,
                        self.camera_matrix,
                        self.distortion_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )

                    if not ok:  
                        self.logger.debug(
                            "Could not solve PnP for detected tag.",
                            throttle_duration_sec=1.0,
                        )
                        continue

                    T_marker_to_robot, R_marker_to_robot = self.camera2robot_tfs(rvec, tvec)  

                    dist_to_block = np.hypot(T_marker_to_robot[0].item(), T_marker_to_robot[1].item())
                    # self.get_logger().info(f'distance to block: {dist_to_block}')
                    if dist_to_block > self.max_block_dist:
                        continue

                    # only add in the visible blocks below 2m away.
                    curr_visible_block_ids.add(candidate_block_id)

                    candidate_pose_stamped = self.create_pose(msg, T_marker_to_robot, R_marker_to_robot)

                    self.detection_queue.append({
                        'stamp': msg.header.stamp,
                        'pose': candidate_pose_stamped,
                        'block_id': candidate_block_id,
                    })

                    for block in self.found_blocks:
                        if block.block_id == candidate_block_id:
                            block.time_seen = self.get_clock().now()
                            break
                            
                
                self.get_logger().info(f'visible block ids: {curr_visible_block_ids}')
                
            else:
                self.logger.debug(
                    "No tags detected in the image.", throttle_duration_sec=1.0
                )

            self.visible_block_ids = curr_visible_block_ids.copy()

            self.publish_markers()

        except Exception as e:
            self.logger.error(f"Error converting image: {e}")

    def process_detection_queue(self):
        still_pending = deque()
        for detection in self.detection_queue:
            stamp = detection['stamp']
            pose = detection['pose']
            block_id = detection['block_id']

            if self.tf_buffer.can_transform('world', 'aruco_31', stamp):
                try:
                    transformed_pose = self.tf_buffer.transform(pose, 'world')

                    found_block = None
                    for block in self.found_blocks:
                        if block.block_id == block_id:
                            found_block = block
                            break

                    if found_block is None:
                        found_block = TrackedBlock(block_id, self.get_clock().now())
                        self.found_blocks.append(found_block)

                    # self.get_logger().info(f'Updating block_id {block_id}')
                    found_block.update(transformed_pose) # alias holds

                except Exception as e:
                    self.get_logger().warn(f'Transform failed even after can_transform: {e}')
            else:
                still_pending.append(detection)

        self.detection_queue = still_pending


    def publish_blocks(self):
        for block in self.found_blocks:

            if block.block_id in self.published_blocks:
                continue

            dt = (self.get_clock().now().nanoseconds - block.time_seen.nanoseconds) / 1e9 # seconds
            if dt < 2: # 2 seconds
                continue

            pub_block = Block()
            block_type = self.id2type.get(block.block_id, None)
            if block_type is None:
                self.get_logger().info(f'block_id: {block.block_id} type is unknown. Not publishing')
                continue

            pub_block.type = block_type 
            pub_block.pose = block.get_averaged_pose(self.get_clock().now().to_msg())

            self.get_logger().info(f'Publishing block_id: {block.block_id} at\
                (x: {pub_block.pose.pose.position.x} \
                y: {pub_block.pose.pose.position.y}) in world frame')
            self.vis_pub.publish(pub_block)
            self.published_blocks.append(block.block_id)

    def create_pose(self, msg, T_marker_to_robot, R_marker_to_robot):
        candidate_pose_stamped = PoseStamped()
        candidate_pose_stamped.header.frame_id = 'aruco_31'
        candidate_pose_stamped.header.stamp = msg.header.stamp

        candidate_pose_stamped.pose.position.x = float(T_marker_to_robot[0])
        candidate_pose_stamped.pose.position.y = float(T_marker_to_robot[1])
        candidate_pose_stamped.pose.position.z = float(T_marker_to_robot[2])

        x, y, z, w = Rotation.from_matrix(R_marker_to_robot).as_quat()
        candidate_pose_stamped.pose.orientation.x = x
        candidate_pose_stamped.pose.orientation.y = y
        candidate_pose_stamped.pose.orientation.z = z
        candidate_pose_stamped.pose.orientation.w = w

        # self.get_logger().info(
        #     f'Pose in frame_id aruco_31: '
        #     f'x: {candidate_pose_stamped.pose.position.x:.3f}, '
        #     f'y: {candidate_pose_stamped.pose.position.y:.3f}, '
        #     f'z: {candidate_pose_stamped.pose.position.z:.3f}'
        # )

        return candidate_pose_stamped

    def camera2robot_tfs(self, rvec, tvec):
        R_marker_to_cam, _ = cv2.Rodrigues(rvec)

        R_image_to_robot_axes = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0],
        ])

        R_cam_to_robot = R_image_to_robot_axes
        R_marker_to_robot = R_cam_to_robot @ R_marker_to_cam

        T_cam_to_robot = np.array([[-0.1], [0], [0]])
        T_marker_in_cam = tvec.reshape(3, 1)
        T_marker_to_robot = R_cam_to_robot @ T_marker_in_cam + T_cam_to_robot

        return T_marker_to_robot, R_marker_to_robot

    def publish_markers(self):
        marker_array = MarkerArray()
        for block in self.found_blocks:
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'found_blocks'
            marker.id = block.block_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = block.get_averaged_pose(self.get_clock().now().to_msg()).pose
            marker.scale.x = 0.1
            marker.scale.y = 0.4
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