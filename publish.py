'''
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class DiagnosticPublisher(Node):
    def __init__(self):
        super().__init__('diagnostic_publisher')
        self.publisher_ = self.create_publisher(DiagnosticArray, '/aeb/fused_data', 10)
        self.timer = self.create_timer(1.0, self.publish_scenarios)

    def publish_scenarios(self):
        diagnostic_msg = DiagnosticArray()

        # Scenario 1: Moving Pedestrian
        pedestrian_status = DiagnosticStatus()
        pedestrian_status.name = 'Moving Pedestrian'
        pedestrian_status.level = b'\x00'  # OK
        pedestrian_status.message = ''
        pedestrian_status.hardware_id = 'sensor_fusion'
        pedestrian_status.values.append(KeyValue(key='class_id', value='0'))  # Pedestrian
        pedestrian_status.values.append(KeyValue(key='position_x', value='10.0'))  # Position
        pedestrian_status.values.append(KeyValue(key='position_y', value='1.0'))
        pedestrian_status.values.append(KeyValue(key='position_z', value='0.0'))
        pedestrian_status.values.append(KeyValue(key='velocity_x', value='1.5'))  # Velocity
        pedestrian_status.values.append(KeyValue(key='velocity_y', value='0.0'))
        pedestrian_status.values.append(KeyValue(key='velocity_z', value='0.0'))
        diagnostic_msg.status.append(pedestrian_status)

        # Scenario 2: Moving Bicyclist
        bicyclist_status = DiagnosticStatus()
        bicyclist_status.name = 'Moving Bicyclist'
        bicyclist_status.level = b'\x00'  # OK
        bicyclist_status.message = ''
        bicyclist_status.hardware_id = 'sensor_fusion'
        bicyclist_status.values.append(KeyValue(key='class_id', value='1'))  # Bicyclist
        bicyclist_status.values.append(KeyValue(key='position_x', value='20.0'))
        bicyclist_status.values.append(KeyValue(key='position_y', value='-2.0'))
        bicyclist_status.values.append(KeyValue(key='position_z', value='0.0'))
        bicyclist_status.values.append(KeyValue(key='velocity_x', value='3.0'))
        bicyclist_status.values.append(KeyValue(key='velocity_y', value='0.0'))
        bicyclist_status.values.append(KeyValue(key='velocity_z', value='0.0'))
        diagnostic_msg.status.append(bicyclist_status)

        # Scenario 3: Stationary Bicyclist
        stationary_bicyclist_status = DiagnosticStatus()
        stationary_bicyclist_status.name = 'Stationary Bicyclist'
        stationary_bicyclist_status.level = b'\x00'  # OK
        stationary_bicyclist_status.message = ''
        stationary_bicyclist_status.hardware_id = 'sensor_fusion'
        stationary_bicyclist_status.values.append(KeyValue(key='class_id', value='1'))  # Bicyclist
        stationary_bicyclist_status.values.append(KeyValue(key='position_x', value='15.0'))
        stationary_bicyclist_status.values.append(KeyValue(key='position_y', value='0.5'))
        stationary_bicyclist_status.values.append(KeyValue(key='position_z', value='0.0'))
        stationary_bicyclist_status.values.append(KeyValue(key='velocity_x', value='0.0'))
        stationary_bicyclist_status.values.append(KeyValue(key='velocity_y', value='0.0'))
        stationary_bicyclist_status.values.append(KeyValue(key='velocity_z', value='0.0'))
        diagnostic_msg.status.append(stationary_bicyclist_status)

        # Scenario 4: Car and Pedestrian
        car_and_pedestrian_status = DiagnosticStatus()
        car_and_pedestrian_status.name = 'Car and Pedestrian'
        car_and_pedestrian_status.level = b'\x00'  # OK
        car_and_pedestrian_status.message = ''
        car_and_pedestrian_status.hardware_id = 'sensor_fusion'
        car_and_pedestrian_status.values.append(KeyValue(key='class_id', value='2'))  # Car
        car_and_pedestrian_status.values.append(KeyValue(key='position_x', value='8.0'))
        car_and_pedestrian_status.values.append(KeyValue(key='position_y', value='1.2'))
        car_and_pedestrian_status.values.append(KeyValue(key='position_z', value='0.0'))
        car_and_pedestrian_status.values.append(KeyValue(key='velocity_x', value='5.0'))
        car_and_pedestrian_status.values.append(KeyValue(key='velocity_y', value='0.0'))
        car_and_pedestrian_status.values.append(KeyValue(key='velocity_z', value='0.0'))
        diagnostic_msg.status.append(car_and_pedestrian_status)

        # Scenario 5: Overlap 50%
        overlap_50_status = DiagnosticStatus()
        overlap_50_status.name = 'Overlap 50%'
        overlap_50_status.level = b'\x00'  # OK
        overlap_50_status.message = ''
        overlap_50_status.hardware_id = 'sensor_fusion'
        overlap_50_status.values.append(KeyValue(key='class_id', value='0'))  # Pedestrian
        overlap_50_status.values.append(KeyValue(key='position_x', value='6.0'))
        overlap_50_status.values.append(KeyValue(key='position_y', value='0.75'))
        overlap_50_status.values.append(KeyValue(key='position_z', value='0.0'))
        overlap_50_status.values.append(KeyValue(key='velocity_x', value='0.0'))
        overlap_50_status.values.append(KeyValue(key='velocity_y', value='0.0'))
        overlap_50_status.values.append(KeyValue(key='velocity_z', value='0.0'))
        diagnostic_msg.status.append(overlap_50_status)

        # Scenario 6: Overlap -50%
        overlap_negative_50_status = DiagnosticStatus()
        overlap_negative_50_status.name = 'Overlap -50%'
        overlap_negative_50_status.level = b'\x00'  # OK
        overlap_negative_50_status.message = ''
        overlap_negative_50_status.hardware_id = 'sensor_fusion'
        overlap_negative_50_status.values.append(KeyValue(key='class_id', value='0'))  # Pedestrian
        overlap_negative_50_status.values.append(KeyValue(key='position_x', value='6.0'))
        overlap_negative_50_status.values.append(KeyValue(key='position_y', value='-0.75'))
        overlap_negative_50_status.values.append(KeyValue(key='position_z', value='0.0'))
        overlap_negative_50_status.values.append(KeyValue(key='velocity_x', value='0.0'))
        overlap_negative_50_status.values.append(KeyValue(key='velocity_y', value='0.0'))
        overlap_negative_50_status.values.append(KeyValue(key='velocity_z', value='0.0'))
        diagnostic_msg.status.append(overlap_negative_50_status)

        # Publish all scenarios
        self.publisher_.publish(diagnostic_msg)
        self.get_logger().info('Published diagnostic data for multiple scenarios.')

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class DiagnosticPublisher(Node):
    def __init__(self):
        super().__init__('diagnostic_publisher')
        self.publisher_ = self.create_publisher(DiagnosticArray, '/aeb/fused_data', 10)
        self.timer = self.create_timer(1.0, self.publish_scenarios)

    def publish_scenarios(self):
        diagnostic_msg = DiagnosticArray()

        # Aggressive braking scenario: Object is very close and moving toward the vehicle
        braking_object = DiagnosticStatus()
        braking_object.name = 'Aggressive Braking Object'
        braking_object.level = b'\x00'
        braking_object.hardware_id = 'sensor_fusion'
        braking_object.values.append(KeyValue(key='class_id', value='2'))  # Car
        braking_object.values.append(KeyValue(key='position_x', value='5.0'))  # Very close
        braking_object.values.append(KeyValue(key='position_y', value='0.0'))
        braking_object.values.append(KeyValue(key='position_z', value='0.0'))
        braking_object.values.append(KeyValue(key='velocity_x', value='-10.0'))  # Moving toward the vehicle
        braking_object.values.append(KeyValue(key='velocity_y', value='0.0'))
        braking_object.values.append(KeyValue(key='velocity_z', value='0.0'))
        diagnostic_msg.status.append(braking_object)

        # Publish the diagnostic message
        self.publisher_.publish(diagnostic_msg)
        self.get_logger().info('Published aggressive braking scenario.')

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
