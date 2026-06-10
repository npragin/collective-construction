import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class BlockScan(Node):

    def __init__(self):
        super().__init__('blockscan')

        self.sub_color = self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)

        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.bridge = CvBridge()
        self.block_detect = False
        self.depth_image = None

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def camera_callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info(f'Error converting image: {e}')
            return
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        orange_hsv_lower = np.array([0.0, 128.0, 200.0])
        orange_hsv_higher = np.array([15.0, 255.0, 255.0])

        mask = cv2.inRange(hsv, orange_hsv_lower, orange_hsv_higher)

        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)

        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        # Default behavior: rotate and search
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.5

        self.block_detect = False

        if contours:

            largest_contour = max(contours,key=cv2.contourArea)

            area = cv2.contourArea(largest_contour)

            if area > 500:

                M = cv2.moments(largest_contour)

                if M['m00'] > 0:

                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])

                    height, width, _ = cv_image.shape

                    frame_center = width / 2
                    error = frame_center - cX

                    angular = error * 0.003
                    angular = max(min(angular, 0.5), -0.5)

                    if abs(error) > 20:
                        twist.twist.linear.x = 0.0
                        twist.twist.angular.z = angular

                    else:
                        if self.depth_image is not None:
                            try:
                                depth = self.depth_image[cY, cX]
                                # Ignore invalid depth
                                if depth > 0:
                                    # Distance thresholds in mm
                                    if depth > 500:
                                        # Move forward
                                        twist.twist.linear.x = 0.10
                                        twist.twist.angular.z = 0.0

                                    elif depth < 300:
                                        # Too close
                                        twist.twist.linear.x = -0.05
                                        twist.twist.angular.z = 0.0

                                    else:
                                        # Desired pickup range
                                        twist.twist.linear.x = 0.0
                                        twist.twist.angular.z = 0.0

                                        self.block_detect = True

                                        self.get_logger().info(f'Block detected at {depth} mm')

                            except IndexError:
                                pass

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    scanner = BlockScan()
    rclpy.spin(scanner)
    scanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()