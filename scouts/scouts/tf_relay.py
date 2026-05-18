# tf_relay.py
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        self.declare_parameter('robot_name', '')
        robot_name = self.get_parameter('robot_name').value

        self.sub = self.create_subscription(
            TFMessage,
            f'/{robot_name}/tf',
            self.callback,
            10
        )
        self.pub = self.create_publisher(TFMessage, '/tf', 10)
        self.robot_name = robot_name

    def callback(self, msg):
        for t in msg.transforms:
            # prefix the frame IDs
            t.header.frame_id = f'{self.robot_name}/{t.header.frame_id}'
            t.child_frame_id = f'{self.robot_name}/{t.child_frame_id}'
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(TFRelay())

if __name__ == '__main__':
    main()