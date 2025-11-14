#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

class PWMSimNode(Node):
    def __init__(self):
        super().__init__('pwm_sim_node')

        # Subscribe to messages from the control layer
        self.subscription = self.create_subscription(
            AckermannDrive,
            '/driveData',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused warnings

        self.get_logger().info("PWM Simulation Node started. Waiting for AckermannDrive messages...")

    def listener_callback(self, msg):
        accel = msg.acceleration
        steering = msg.steering_angle

        # Determine direction
        direction = "FORWARD" if accel >= 0 else "REVERSE"
        accel = abs(accel)

        # Map acceleration to PWM duty cycle (0–100%)
        accel_clamped = min(max(accel, 0.0), 3200.0)
        duty_cycle = accel_clamped / 3200.0 * 100.0

        # Simulated average voltage (assuming 5V power supply)
        voltage = duty_cycle / 100.0 * 5.0

        # Print result
        self.get_logger().info(
            f"Recv AckermannDrive → PWM Duty: {duty_cycle:.1f}%, Avg Voltage: {voltage:.2f} V, Dir: {direction}, Steering: {steering:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PWMSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
