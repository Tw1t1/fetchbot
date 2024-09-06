import rclpy
import pickle
import os
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from fetchbot_interfaces.msg import Force
from threading import Lock

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
            callback_group=callback_group)
        self.save_subscription = self.create_subscription(
            String,
            '/save_scan',
            self.save_scan_callback,
            10,
            callback_group=callback_group)
        self.pub_subscription = self.create_subscription(
            String,
            '/pub_scan',
            self.pub_scan_callback,
            10,
            callback_group=callback_group)
        self.force_subscription = self.create_subscription(
            Force,
            '/force',
            self.force_callback,
            10,
            callback_group=callback_group)
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.srv = self.create_service(Trigger, 'test_force', self.test_force_callback, callback_group=callback_group)

        self.latest_scan = None
        self.latest_force = 0.0
        self.log_file = 'feel_force_results.log'
        self.lock = Lock()

        self.scan_files = [
            'short_25', 'short_50', 'short_75', 'short_100', 'short_125', 'short_150',
            'long_25', 'long_50', 'long_75', 'long_100', 'long_125', 'long_150'
        ]
        self.current_scan_index = 0
        self.publishing_scans = False
        self.log_timer = None
        self.publish_timer = None

    def scan_callback(self, msg):
        self.latest_scan = msg

    def save_scan_callback(self, msg):
        if self.latest_scan is None:
            self.get_logger().warn('No scan data available to save')
            return
        filename = msg.data
        with open(filename, 'wb') as f:
            pickle.dump(self.latest_scan, f)
        self.get_logger().info(f'Saved scan to file: {filename}')

    def pub_scan_callback(self, msg):
        filename = msg.data
        self.publish_scan_file(filename)

    def force_callback(self, msg):
        with self.lock:
            if msg.magnitude > 0:
                self.latest_force = msg.magnitude
        self.get_logger().debug(f'Received force: {self.latest_force}')

    def log_results(self, filename):
        with self.lock:
            force_to_log = self.latest_force
            self.latest_force = 0.0  # Reset force after logging
        with open(self.log_file, 'a') as f:
            f.write(f"Scan: {filename}, Force Magnitude: {force_to_log}\n")
        self.get_logger().info(f'Logged results for scan: {filename}, Force Magnitude: {force_to_log}')
        if self.log_timer:
            self.log_timer.cancel()
            self.log_timer = None

    def publish_scan_file(self, filename):
        if not os.path.exists(filename):
            self.get_logger().warn(f'File not found: {filename}')
            return
        with open(filename, 'rb') as f:
            scan_msg = pickle.load(f)
        self.publisher.publish(scan_msg)
        self.get_logger().info(f'Published scan from file: {filename}')
        
        # Schedule logging after a delay
        if self.log_timer:
            self.log_timer.cancel()
        self.log_timer = self.create_timer(0.5, lambda: self.log_results(filename))

    def test_force_callback(self, request, response):
        if not self.publishing_scans:
            self.get_logger().info('Received test_force service call')
            self.current_scan_index = 0
            self.publishing_scans = True
            self.publish_next_scan()
            response.success = True
            response.message = f"Started publishing {len(self.scan_files)} scan files"
        else:
            response.success = False
            response.message = "Already publishing scans"
        return response

    def publish_next_scan(self):
        if self.current_scan_index < len(self.scan_files):
            filename = self.scan_files[self.current_scan_index]
            self.publish_scan_file(filename)
            self.current_scan_index += 1
            # Schedule next publication
            if self.publish_timer:
                self.publish_timer.cancel()
            self.publish_timer = self.create_timer(1.0, self.publish_next_scan)
        else:
            self.publishing_scans = False
            self.get_logger().info("Finished publishing all scans")
            if self.publish_timer:
                self.publish_timer.cancel()
                self.publish_timer = None

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(test_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()