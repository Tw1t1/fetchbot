import rclpy, time
from rclpy.node import Node
from std_msgs.msg import String
from fetchbot_interfaces.msg import Collision

# Conditional import for RPi.GPIO
try:
    import RPi.GPIO as GPIO
except ImportError:
    from unittest.mock import MagicMock
    GPIO = MagicMock()

class IndicatorNode(Node):
    def __init__(self):
        super().__init__('indicator')
        self.subscription = self.create_subscription(
            Collision,
            'collision',
            self.collision_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'follow_ball/status',
            self.follow_ball_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'orient_home/status',
            self.orient_home_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'low_battery',
            self.low_battery_callback,
            10)
        self.publisher = self.create_publisher(String, 'indicate', 10)
        
        self.status = {
            'collision': {'active': False, 'timestamp': 0, 'color': (255, 0, 0)},  # Red
            'follow_ball': {'active': False, 'timestamp': 0, 'color': (0, 0, 255)},  # Blue
            'orient_home': {'active': False, 'timestamp': 0, 'color': (0, 255, 0)},  # Green
            'low_battery': {'active': False, 'timestamp': 0, 'color': (255, 165, 0)}  # Orange
        }
        
        self.timeout = 0.3  # 0.3 seconds timeout
        
        # Set up GPIO for Keyes 3-Color RGB LED
        self.R, self.G, self.B = 19, 26, 13
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        GPIO.cleanup([self.R, self.G, self.B])
        
        GPIO.setup(self.R, GPIO.OUT)
        GPIO.setup(self.G, GPIO.OUT)
        GPIO.setup(self.B, GPIO.OUT)
        
        # For common cathode, we use GPIO.PWM directly
        self.pwm_r = GPIO.PWM(self.R, 100)
        self.pwm_g = GPIO.PWM(self.G, 100)
        self.pwm_b = GPIO.PWM(self.B, 100)
        
        self.pwm_r.start(0)
        self.pwm_g.start(0)
        self.pwm_b.start(0)
        
        # Create a timer for updating LED
        self.create_timer(0.1, self.update_led)

    def collision_callback(self, msg):
        self.update_status('collision')
    
    def follow_ball_callback(self, msg):
        if msg.data == "UNREACHABLE":
            self.update_status('follow_ball')

    def orient_home_callback(self, msg):
        self.update_status('orient_home')

    def low_battery_callback(self, msg):
        self.update_status('low_battery')
    
    def update_status(self, status_key):
        self.status[status_key]['active'] = True
        self.status[status_key]['timestamp'] = time.time()
        self.publish_status()

    def publish_status(self):
        msg = String()
        active_statuses = [key for key, value in self.status.items() if value['active']]
        msg.data = ' '.join(active_statuses)
        self.publisher.publish(msg)

    def update_led(self):
        current_time = time.time()
        active_colors = []
        
        for status, data in self.status.items():
            if data['active']:
                if current_time - data['timestamp'] > self.timeout:
                    data['active'] = False
                else:
                    active_colors.append(data['color'])
        
        if not active_colors:
            self.set_color(0, 0, 0)  # Turn off LED if no active status
        elif len(active_colors) == 1:
            self.set_color(*active_colors[0])
        else:
            # Cycle through active colors
            color_index = int(current_time * 5) % len(active_colors)
            self.set_color(*active_colors[color_index])

    def set_color(self, r, g, b):
        # For common cathode, higher duty cycle means brighter LED
        self.pwm_r.ChangeDutyCycle(r / 255 * 100)
        self.pwm_g.ChangeDutyCycle(g / 255 * 100)
        self.pwm_b.ChangeDutyCycle(b / 255 * 100)

    def destroy_node(self):
        self.pwm_r.stop()
        self.pwm_g.stop()
        self.pwm_b.stop()
        GPIO.cleanup([self.R, self.G, self.B])
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    indicator = IndicatorNode()
    try:
        rclpy.spin(indicator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            indicator.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()