import rclpy
from rclpy.node import Node
from geometry_msgs.msg      import Point
from sensor_msgs.msg import Image
import time
from datetime import datetime

class MessageCounter(Node):
    def __init__(self):
        super().__init__('message_counter')
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.listener_callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.subscription  # prevent unused variable warning
        self.message_count = 0
        self.last_time = time.time()

    def listener_callback(self, msg):
        self.message_count += 1
        current_time = time.time()
        time_diff = current_time - self.last_time
        if time_diff >= 1.0:
            current_datetime = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.get_logger().info(f'Time: {current_datetime}, '
                                   f'Msgs per sec: {self.message_count}, '
                                   f'Time diff: {time_diff:.3f} sec')
            self.message_count = 0
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    message_counter = MessageCounter()
    rclpy.spin(message_counter)
    message_counter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
