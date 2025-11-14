#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

class DacSimNode(Node):
    def __init__(self):
        super().__init__('dac_sim_node')
        self.subscription = self.create_subscription(
            AckermannDrive,
            '/driveData',
            self.listener_callback,
            10)
        self.subscription
        self.get_logger().info("DAC Simulation Node started. Waiting for AckermannDrive messages...")

    def listener_callback(self, msg):
        accel = msg.acceleration
        steering = msg.steering_angle

        # Determine direction
        direction = "FORWARD" if accel >= 0 else "REVERSE"
        accel = abs(accel)

        # Map the acceleration to a 0–4095 DAC output
        accel_clamped = min(max(accel, 0.0), 3200.0)
        dac_value = int(accel_clamped / 3200.0 * 4095.0)
        voltage = dac_value / 4095.0 * 5.0

        self.get_logger().info(
            f"Recv AckermannDrive → DAC Output: {dac_value} (≈ {voltage:.2f} V), Dir: {direction}, Steering: {steering:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DacSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

