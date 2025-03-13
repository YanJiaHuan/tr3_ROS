import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import time

class BluetoothMagnetNode(Node):

    def __init__(self):
        super().__init__('bluetooth_magnet_node')

        # Subscription for control commands
        self.subscription = self.create_subscription(
            Bool,
            '/electromagnet_control',
            self.listener_callback,
            10)

        # Publisher for the magnet state
        self.state_publisher = self.create_publisher(Bool, '/electromagnet_state', 10)

        # Declare parameters for serial port settings
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

        # Internal state for the magnet (assume OFF by default)
        self.magnet_state = False

        # Timer for publishing magnet state periodically
        self.frequency = 10
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.publish_state_callback)

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
            
            # Update the internal magnet state, but do not publish it here
            self.magnet_state = msg.data
        except Exception as e:
            self.get_logger().error(f'Serial transmission failed: {e}')

    def publish_state_callback(self):
        # Publish the current magnet state on a timer
        state_msg = Bool()
        state_msg.data = self.magnet_state
        self.state_publisher.publish(state_msg)
        self.get_logger().debug(f'Published magnet state: {"ON" if self.magnet_state else "OFF"}')

    def destroy_node(self):
        # Before closing, ensure the magnet is turned off
        if self.ser is not None:
            try:
                self.ser.write('0'.encode())
                self.get_logger().info('Sent command to turn OFF magnet.')
                # Update the state for the publisher as well
                self.magnet_state = False
            except Exception as e:
                self.get_logger().error(f'Failed to send turn off command: {e}')
            self.ser.close()
            self.get_logger().info('Closed serial connection.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    bluetooth_magnet_node = BluetoothMagnetNode()

    try:
        rclpy.spin(bluetooth_magnet_node)
    except KeyboardInterrupt:
        bluetooth_magnet_node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        bluetooth_magnet_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
