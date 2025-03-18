import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class sub(Node):
    def __init__(self):
        super().__init__('sub')
        self.sub_ = self.create_subscription(Float32MultiArray, 'joint_angles', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received joint angles: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = sub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()