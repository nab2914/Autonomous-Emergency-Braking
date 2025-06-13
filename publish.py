import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class DiagnosticPublisher(Node):
    def __init__(self):
        super().__init__('diagnostic_publisher')
        self.publisher_ = self.create_publisher(DiagnosticArray, '/aeb/fused_data', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        diagnostic_msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = 'Object Data'
        status.level = b'\x00'  # Correctly encode the level as a single byte (0 = OK)
        status.message = ''
        status.hardware_id = 'sensor_fusion'

        # Add key-value pairs
        status.values.append(KeyValue(key='class_id', value='0'))
        status.values.append(KeyValue(key='position_x', value='50.0'))
        status.values.append(KeyValue(key='position_y', value='0.0'))
        status.values.append(KeyValue(key='position_z', value='0.0'))
        status.values.append(KeyValue(key='velocity_x', value='0.0'))
        status.values.append(KeyValue(key='velocity_y', value='0.0'))
        status.values.append(KeyValue(key='velocity_z', value='0.0'))

        diagnostic_msg.status.append(status)
        self.publisher_.publish(diagnostic_msg)
        self.get_logger().info('Published DiagnosticArray message')

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
