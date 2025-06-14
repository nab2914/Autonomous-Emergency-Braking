#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_fusion_msgs.msg import FusedObjectArray  # Updated input from sensor fusion
from vehiclecontrol.msg import Control  # Updated output message

class AEBControllerNode(Node):
    def __init__(self):
        super().__init__('aeb_controller1')

        # Publisher for vehicle control commands
        self.control_pub = self.create_publisher(Control, '/vehiclecontrol', 10)

        # Subscriber for fused sensor data
        self.fused_data_sub = self.create_subscription(
            FusedObjectArray, '/fused_objects', self.fused_data_callback, 10
        )

        # Constants and parameters
        self.max_throttle = 100.0  # Max throttle percentage
        self.max_brake = 100.0     # Max brake percentage
        self.target_speed = 8.33333   # Target speed in m/s
        self.target_stop_distance = 6.0
        self.stop_tolerance = 2.0  # ±2m tolerance
        self.min_decel = 5.0       # Min deceleration rate in m/s²
        self.max_decel = 8.0       # Max deceleration rate in m/s²
        self.safe_ttc_threshold = 8.0  # Updated: practical upper TTC for safe acceleration
        self.emergency_ttc = 1.0   # Threshold TTC for emergency braking (seconds)
        self.warning_ttc = 3.0     # Threshold TTC for warning braking (seconds)
        self.vehicle_width = 1.5   # Width of the BAJA Car in meters
        

        # State variables
        self.current_distance = float('inf')
        self.relative_velocity = 0.0
        self.current_ttc = float('inf')
        self.traveled_distance = 0.0
        self.target_class_id = None
        self.target_position = None
        self.target_width = 1.8   # Default width of the target object (can be adjusted)

    def fused_data_callback(self, msg):
        """
        Callback for processing fused sensor data.
        Data format: FusedObjectArray containing:
            - int32 class_id
            - geometry_msgs/Point position
            - geometry_msgs/Vector3 velocity
        """
        try:
            nearest_distance = float('inf')
            nearest_velocity = None
            nearest_obj_class_id = None
            nearest_obj_position = None

            for fused_object in msg.objects:
                obj_class_id = fused_object.class_id
                obj_position = fused_object.position
                obj_velocity = fused_object.velocity

                # Calculate distance to object
                distance = (obj_position.x**2 + obj_position.y**2 + obj_position.z**2)**0.5

                # Update nearest object information
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_velocity = obj_velocity
                    nearest_obj_class_id = obj_class_id
                    nearest_obj_position = obj_position

            # Update state variables
            self.current_distance = nearest_distance
            self.relative_velocity = nearest_velocity.x  # Assuming relative velocity is along x-axis
            self.current_ttc = self.calculate_ttc(nearest_distance, nearest_velocity.x)
            self.target_class_id = nearest_obj_class_id
            self.target_position = nearest_obj_position

            # Process the data for control
            self.control_aeb()
        except Exception as e:
            self.get_logger().error(f"Error in fused_data_callback: {e}")
            return  # Ensure we stop further execution in case of error

        self.get_logger().info(
            f"Processed Data - Class ID: {self.target_class_id}, "
            f"Position: ({self.target_position.x:.2f}, {self.target_position.y:.2f}, {self.target_position.z:.2f}), "
            f"Velocity: {self.relative_velocity:.2f} m/s, "
            f"Distance: {self.current_distance:.2f} m, "
            f"TTC: {self.current_ttc:.2f} s"
        )

    def calculate_overlap(self, ego_position, target_position):
        """
        Calculate lateral overlap between ego vehicle and target object.
        """
        lateral_distance = abs(ego_position.y - target_position.y)

        # Overlap calculation (percentage overlap)
        overlap = max(0, (self.vehicle_width + self.target_width - lateral_distance) / (self.vehicle_width + self.target_width))
        return min(overlap, 1.0)  # Constrain overlap to [0, 1]

    def calculate_ttc(self, distance, relative_velocity):
        """
        Calculate Time-to-Collision (TTC).
        """
        if relative_velocity != 0 and distance > 0:  # Handle both positive and negative velocities
            return distance / abs(relative_velocity)  # Use absolute value of relative velocity
        return float('inf')  # Return infinity if velocity is zero or distance is non-positive

    def control_aeb(self):
        """
        Main AEB logic: Determines throttle/brake commands based on zones and scenarios.
        """
        # Initialize control message
        control_msg = Control()
        control_msg.steering = 0.0  # Default steering angle
        control_msg.latswitch = 0   # Default lateral switch state
        control_msg.longswitch = 1  # Enable longitudinal control switch

        # Debugging: Log the current TTC and relative velocity
        self.get_logger().info(
            f"Evaluating Control AEB - TTC: {self.current_ttc:.2f}, Relative Velocity: {self.relative_velocity:.2f}, Distance: {self.current_distance:.2f}"
        )

        # Zone 1: Acceleration Zone
        if self.current_ttc > self.safe_ttc_threshold: # No obstacle detected (TTC is infinite)
            if self.relative_velocity < self.target_speed:
                control_msg.throttle = self.max_throttle
                control_msg.brake = 0.0
                self.publish_control(control_msg)
                self.get_logger().info(f"Safe TTC ({self.current_ttc:.2f}s > {self.safe_ttc_threshold}s): Accelerating. Throttle: {control_msg.throttle:.2f}%")
                self.get_logger().info(f"Zone 1: Accelerating. Throttle: {control_msg.throttle:.2f}%")
            return

        # Calculate overlap factor
        ego_position = self.target_position  # Ego is represented by its position
        overlap = self.calculate_overlap(ego_position, self.target_position)
        self.get_logger().info(f"Overlap: {overlap:.2f}, Target Class ID: {self.target_class_id}")

        # Zone 2: Braking Zone
        if self.current_ttc < self.warning_ttc:
            if self.current_ttc < self.emergency_ttc:
                # Emergency braking
                control_msg.throttle = 0.0
                control_msg.brake = self.max_brake
                self.publish_control(control_msg)
                self.get_logger().info(f"Zone 2: Emergency Brake! Brake: {control_msg.brake:.2f}%, TTC: {self.current_ttc:.2f}")
            else:
                # Gradual braking with overlap consideration
                control_msg.throttle = 0.0
                control_msg.brake = self.calculate_brake_force(self.current_ttc, overlap, self.target_class_id)
                self.publish_control(control_msg)
                self.get_logger().info(f"Zone 2: Gradual Brake. Brake: {control_msg.brake:.2f}%, TTC: {self.current_ttc:.2f}")
            return

        # Zone 3: Stopping Zone
        if (self.target_stop_distance - self.stop_tolerance <= self.current_distance <= self.target_stop_distance + self.stop_tolerance):
            control_msg.throttle = 0.0
            control_msg.brake = self.max_brake
            self.publish_control(control_msg)
            self.get_logger().info("Zone 3: Stopping achieved within target zone.")
            return

    def calculate_brake_force(self, ttc, overlap, class_id):
        """
        Calculate appropriate brake force based on TTC, overlap factor, and object class ID.
        """
        brake_factor = 1.0 - overlap  # Reduce brake force based on overlap

        # Adjust brake factor based on object class ID
        if class_id == 2:  # Car
            brake_factor *= 1.0  # Full brake force for cars
        elif class_id == 0:  # Pedestrian
            brake_factor *= 0.8  # Reduced brake force for pedestrians
        elif class_id == 1:  # Bicycle
            brake_factor *= 0.6  # Reduced brake force for bicycles

        # Linear scaling of brake force based on TTC
        return self.max_brake * brake_factor * (1.0 - (ttc / self.warning_ttc))

    def publish_control(self, control_msg):
        """
        Publishes control commands to the vehicle control topic.
        """
        self.get_logger().info(
            f"Publishing Control Command - Steering: {control_msg.steering}, "
            f"Throttle: {control_msg.throttle}, Brake: {control_msg.brake}, "
            f"LongSwitch: {control_msg.longswitch}, LatSwitch: {control_msg.latswitch}"
        )
        self.control_pub.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AEBControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()