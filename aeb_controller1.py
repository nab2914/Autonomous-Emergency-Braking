#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Vector3
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class AEBControllerNode(Node):
    def __init__(self):
        super().__init__('aeb_controller')

        # Publishers for control commands
        self.throttle_pub = self.create_publisher(Float32, '/control/throttle', 10)
        self.brake_pub = self.create_publisher(Float32, '/control/brake', 10)

        # Subscriber for fused sensor data
        self.fused_data_sub = self.create_subscription(
            DiagnosticArray, '/aeb/fused_data', self.fused_data_callback, 10
        )

        # Constants and parameters
        self.max_throttle = 100.0  # Max throttle percentage
        self.max_brake = 100.0     # Max brake percentage
        self.target_speed = 30.0   # Target speed in km/h
        self.target_stop_distance = 6.0
        self.stop_tolerance = 2.0  # ±2m tolerance
        self.min_decel = 5.0       # Min deceleration rate in m/s²
        self.max_decel = 8.0       # Max deceleration rate in m/s²
        self.emergency_ttc = 1.0   # Threshold TTC for emergency braking (seconds)
        self.warning_ttc = 3.0     # Threshold TTC for warning braking (seconds)

        # State variables
        self.current_distance = float('inf')
        self.relative_velocity = 0.0
        self.current_ttc = float('inf')
        self.traveled_distance = 0.0

    def fused_data_callback(self, msg):
        """
        Callback for processing fused sensor data.
        Data format: DiagnosticArray containing:
            - int32 class_id
            - string class_name
            - geometry_msgs/Point position
            - geometry_msgs/Vector3 velocity
        """
        try:
            if not msg.status or len(msg.status) == 0:
                self.get_logger().error("Invalid fused_data received.")
                return

            nearest_distance = float('inf')
            nearest_velocity = Vector3()
            nearest_obj_id = None

            for status in msg.status:
                obj_id = int(status.values[0].value)  # Extract class_id
                obj_name = status.values[1].value    # Extract class_name
                obj_position = Point(
                    float(status.values[2].value),
                    float(status.values[3].value),
                    float(status.values[4].value)
                )
                obj_velocity = Vector3(
                    float(status.values[5].value),
                    float(status.values[6].value),
                    float(status.values[7].value)
                )

                # Calculate distance to object
                distance = (obj_position.x**2 + obj_position.y**2 + obj_position.z**2)**0.5

                # Update nearest object information
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_velocity = obj_velocity
                    nearest_obj_id = obj_id

            # Update state variables
            self.current_distance = nearest_distance
            self.relative_velocity = nearest_velocity.x  # Assuming relative velocity is along x-axis
            self.current_ttc = self.calculate_ttc(nearest_distance, nearest_velocity.x)

            # Process the data for control
            self.control_aeb()
        except Exception as e:
            self.get_logger().error(f"Error in fused_data_callback: {e}")

    def calculate_ttc(self, distance, relative_velocity):
        """
        Calculate Time-to-Collision (TTC).
        """
        if relative_velocity > 0 and distance > 0:
            return distance / relative_velocity
        return float('inf')

    def control_aeb(self):
        """
        Main AEB logic: Determines throttle/brake commands based on zones and scenarios.
        """
        # Zone 1: Acceleration Zone
        if self.traveled_distance < 100:  # First 100m
            if self.relative_velocity < self.target_speed:
                throttle_cmd = self.max_throttle
                self.publish_control(throttle_cmd, 0.0)
                self.get_logger().info(f"Zone 1: Accelerating. Throttle: {throttle_cmd:.2f}%")
                return

        # Zone 2: Braking Zone
        if self.current_ttc < self.warning_ttc:
            if self.current_ttc < self.emergency_ttc:
                # Emergency braking
                brake_cmd = self.max_brake
                self.publish_control(0.0, brake_cmd)
                self.get_logger().info(f"Zone 2: Emergency Brake! Brake: {brake_cmd:.2f}%, TTC: {self.current_ttc:.2f}")
            else:
                # Gradual braking
                brake_cmd = self.calculate_brake_force(self.current_ttc)
                self.publish_control(0.0, brake_cmd)
                self.get_logger().info(f"Zone 2: Gradual Brake. Brake: {brake_cmd:.2f}%, TTC: {self.current_ttc:.2f}")
            return

        # Zone 3: Stopping Zone
        if (self.target_stop_distance - self.stop_tolerance <= self.current_distance <= self.target_stop_distance + self.stop_tolerance):
            brake_cmd = self.max_brake
            self.publish_control(0.0, brake_cmd)
            self.get_logger().info("Zone 3: Stopping achieved within target zone.")
            return

    def calculate_brake_force(self, ttc):
        """
        Calculate appropriate brake force based on TTC.
        """
        # Linear scaling of brake force based on TTC
        return self.max_brake * (1.0 - (ttc / self.warning_ttc))

    def publish_control(self, throttle, brake):
        """
        Publishes throttle and brake commands.
        """
        throttle_msg = Float32()
        throttle_msg.data = throttle
        self.throttle_pub.publish(throttle_msg)

        brake_msg = Float32()
        brake_msg.data = brake
        self.brake_pub.publish(brake_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AEBControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()