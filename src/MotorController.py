import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from SoftwarePWM import SoftwarePWM
import RPi.GPIO as GPIO

# Constants for differential drive
WHEEL_RADIUS = 0.08  # meters, adjust to your robot
BASE_WIDTH = 0.25    # meters, distance between the wheels

ENABLE_PINS = [12, 13, 22, 23]
LEFT_FORWARD_PIN = 26
RIGHT_FORWARD_PIN = 27
LEFT_REVERSE_PIN = 24
RIGHT_REVERSE_PIN = 25

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Subscriber to cmd_vel topic (geometry_msgs/Twist)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10  # QoS profile (queue size)
        )
        
        # Publishers for PWM values for both motors
        self.left_motor_pwm_pub = self.create_publisher(Float32, '/left_motor_throttle', 10)
        self.right_motor_pwm_pub = self.create_publisher(Float32, '/right_motor_throttle', 10)

        GPIO.setmode(GPIO.BCM)
        for pin in ENABLE_PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self.leftForwardMotorSignal = SoftwarePWM(LEFT_FORWARD_PIN)
        self.rightForwardMotorSignal = SoftwarePWM(RIGHT_FORWARD_PIN)
        self.leftReverseMotorSignal = SoftwarePWM(LEFT_REVERSE_PIN)
        self.rightReverseMotorSignal = SoftwarePWM(RIGHT_REVERSE_PIN)

        self.enable_motors(True)

    def enable_motors(self, enabled: bool):
        for pin in ENABLE_PINS:
            if enabled:
                GPIO.output(pin, GPIO.HIGH)
            else:
                GPIO.output(pin, GPIO.LOW)

    def cmd_vel_callback(self, msg: Twist):
        # Extract linear and angular velocities from the Twist message
        linear_velocity = msg.linear.x  # m/s
        angular_velocity = msg.angular.z  # rad/s

        # Compute the left and right wheel speeds (linear velocities)
        left_wheel_speed = (linear_velocity - (BASE_WIDTH / 2) * angular_velocity) / WHEEL_RADIUS
        right_wheel_speed = (linear_velocity + (BASE_WIDTH / 2) * angular_velocity) / WHEEL_RADIUS

        self.publish_throttle(left_wheel_speed, right_wheel_speed)
        self.apply_throttle(left_wheel_speed, right_wheel_speed)
        
    def publish_throttle(self, left_throttle, right_throttle):
        # Publish the PWM values for both motors
        left_throttle_msg = Float32()
        left_throttle_msg.data = left_throttle
        self.left_motor_pwm_pub.publish(left_throttle_msg)

        right_throttle_msg = Float32()
        right_throttle_msg.data = right_throttle
        self.right_motor_pwm_pub.publish(right_throttle_msg)

    def apply_throttle(self, left_throttle, right_throttle):
        if left_throttle > 0:
            self.leftForwardMotorSignal.set_target_value(left_throttle)
            self.leftReverseMotorSignal.set_target_value(0)
        else:
            self.leftForwardMotorSignal.set_target_value(0)
            self.leftReverseMotorSignal.set_target_value(left_throttle)

        if right_throttle > 0:
            self.rightForwardMotorSignal.set_target_value(right_throttle)
            self.rightReverseMotorSignal.set_target_value(0)
        else:
            self.rightForwardMotorSignal.set_target_value(0)
            self.rightReverseMotorSignal.set_target_value(right_throttle)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    # Spin the node to keep it running and handle subscriptions
    rclpy.spin(motor_controller)

    # Destroy the node (clean up)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
