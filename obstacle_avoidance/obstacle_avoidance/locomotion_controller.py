import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading
# from geometry_msgs.msg import TwistStamped    #TODO - use for real robot
from geometry_msgs.msg import Twist #TODO - use for sim robot



class LocomotionControllerNode(Node):
    def __init__(self):
        super().__init__('locomotion_controller')
        self.subscription = self.create_subscription(
            Heading,
            'avoid_runaway_suppressor',
            self.heading_callback,
            10)
        # self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)   #TODO - use for real robot
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)   #TODO - use for sim robot

    def heading_callback(self, msg):
        twist = self.calculate_twist(msg)
        self.publisher.publish(twist)

    #TODO - use for sim robot
    def calculate_twist(self, heading):
        twist = Twist()
        # TODO - update the twist calculation so that the velocity is proportional to the distance and angle
        twist.linear.x = heading.distance / 2.5 #TODO - remove /2.5 for real robot
        twist.angular.z = heading.angle / 3.5 #TODO - remove /2.5 for real robot
        return twist
    
    #TODO - use for real robot
    # def calculate_twist(self, heading):
    #     twist_stamped = TwistStamped()
    #     # TODO - update the twist calculation so that the velocity is proportional to the distance and angle
    #     twist_stamped.twist.linear.x = heading.distance
    #     twist_stamped.twist.angular.z = heading.angle
    #     return twist_stamped


def main(args=None):
    rclpy.init(args=args)
    locomotion_controller = LocomotionControllerNode()
    try:
        rclpy.spin(locomotion_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            locomotion_controller.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()