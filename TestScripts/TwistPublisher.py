import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')  # Initialize node
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # Create publisher for 'cmd_vel' topic
        self.timer = self.create_timer(1.0, self.publish_twist)  # Set up a timer to call the publish_twist method every 1 second
        self.get_logger().info("Twist publisher started")

    def publish_twist(self):
        # Create a Twist message
        msg = Twist()
        msg.linear.x = 0.5  # Set linear velocity along the x-axis
        msg.angular.z = 0.2  # Set angular velocity around the z-axis (turning)
        
        self.publisher_.publish(msg)  # Publish the message
        self.get_logger().info(f'Publishing: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 system
    twist_publisher = TwistPublisher()  # Create a node instance
    rclpy.spin(twist_publisher)  # Spin the node so it keeps running and publishing messages

    # Shutdown the publisher after it's stopped
    twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
