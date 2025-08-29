import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Bug1Node(Node):
    def __init__(self):
        super().__init__('bug1_node')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        twist = Twist()
        # TODO: lógica do Bug1 aqui
        twist.linear.x = 0.2
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Bug1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
