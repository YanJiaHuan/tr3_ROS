import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import time

class BluetoothMagnetNode(Node):

    def __init__(self):
        super().__init__('bluetooth_magnet_node')

        self.subscription = self.create_subscription(
            Bool,
            '/electromagnet_control',
            self.listener_callback,
            10)

        # Declare parameters for the rfcomm port and baud rate
        self.declare_parameter('rfcomm_port', '/dev/rfcomm0')
        self.declare_parameter('baud_rate', 115200)

        port_name = self.get_parameter('rfcomm_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.get_logger().info(f'Attempting to open serial connection on {port_name} at {baud_rate} baud...')
        try:
            self.ser = serial.Serial(port_name, baud_rate)
            time.sleep(2)  # Allow time for the connection to initialize
            self.get_logger().info('Serial connection established successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port_name}: {e}')
            self.ser = None

    def listener_callback(self, msg):
        if self.ser is None:
            self.get_logger().error('Serial connection not available.')
            return

        # Convert Bool message to command ('1' for True/ON, '0' for False/OFF)
        command = '1' if msg.data else '0'
        try:
            self.ser.write(command.encode())
            state_str = 'ON' if msg.data else 'OFF'
            self.get_logger().info(f'Sent electromagnet command: {state_str}')
        except Exception as e:
            self.get_logger().error(f'Serial transmission failed: {e}')

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
            self.get_logger().info('Closed serial connection.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    bluetooth_magnet_node = BluetoothMagnetNode()

    try:
        rclpy.spin(bluetooth_magnet_node)
    except KeyboardInterrupt:
        pass

    bluetooth_magnet_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
