import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import math
import time

class BallMover(Node):
    def __init__(self):
        super().__init__('ball_mover')
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.update_rate = 10 # Hz
        self.timer = self.create_timer(1.0/self.update_rate, self.move_ball)

        self.angle = 0.0
        self.radius = 3.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.z = 0.03  # Assuming ball radius is 3cm, to keep it just touching the ground
        self.angular_speed = 0.1  # radians per second, adjust this to control speed
        
        # Define the hexagon points (x, y)
        self.points = [
            (0.0, 2.0),   # Top
            (-1.0, 1.0),  # Top-left
            (-1.0, -1.0), # Bottom-left
            (0.0, -2.0),  # Bottom
            (1.0, -1.0), # Bottom-right
            (1.0, 1.0),  # Top-right
        ]
        self.current_point = 0
        self.next_point = 1
        self.progress = 0.0
        self.speed = 0.5  # meters per second
        self.last_position = (0.0, 1.0)  # Initialize with the starting position


    
    def move_ball(self):
        # self.move_in_circle()
        self.move_in_hexagon()

    def move_in_hexagon(self):
        current = self.points[self.current_point]
        next = self.points[self.next_point]
        
        # Calculate the distance between current and next point
        distance = math.sqrt((next[0] - current[0])**2 + (next[1] - current[1])**2)
        
        # Calculate the progress increment
        progress_increment = (self.speed / self.update_rate) / distance
        self.progress += progress_increment
        
        if self.progress >= 1.0:
            self.progress = 0.0
            self.current_point = self.next_point
            self.next_point = (self.next_point + 1) % len(self.points)
            current = self.points[self.current_point]
            next = self.points[self.next_point]
        
        # Interpolate position
        x = current[0] + (next[0] - current[0]) * self.progress
        y = current[1] + (next[1] - current[1]) * self.progress
        
        self.last_position = (x, y)  # Update last known position
        
        # Set velocity
        dx = next[0] - current[0]
        dy = next[1] - current[1]
        length = math.sqrt(dx**2 + dy**2)
        linear_x = dx / length * self.speed
        linear_y= dy / length * self.speed

        set_request = self.get_request(x, y, self.z, linear_x, linear_y, 0.0)

        self.future = self.set_state_client.call_async(set_request)
        
        self.get_logger().info(f'Ball position: x={x:.2f}, y={y:.2f}')

    def move_in_circle(self):
        # Calculate new position
        x = self.center_x + self.radius * math.cos(self.angle)
        y = self.center_y + self.radius * math.sin(self.angle)
        
        # Calculate and set velocity (tangent to the circle)
        linear_speed = self.angular_speed * self.radius
        linear_x = -linear_speed * math.sin(self.angle)
        linear_y = linear_speed * math.cos(self.angle)

        set_request = self.get_request(x, y, self.z, linear_x, linear_y, 0.0)
        self.future = self.set_state_client.call_async(set_request)
        
        # Update angle for next iteration
        angle_increment = self.angular_speed / self.update_rate
        self.angle += angle_increment
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi

        self.get_logger().info(f'Ball position: x={x:.2f}, y={y:.2f}, angle={self.angle:.2f}')


    def get_request(self, x, y, z, linear_x, linear_y, linear_z):
        set_request = SetEntityState.Request()
        set_request.state = EntityState()
        set_request.state.name = 'ball'
        
        set_request.state.pose.position.x = x
        set_request.state.pose.position.y = y
        set_request.state.pose.position.z = z
        
        # Set orientation (assuming the ball doesn't rotate)
        set_request.state.pose.orientation.x = 0.0
        set_request.state.pose.orientation.y = 0.0
        set_request.state.pose.orientation.z = 0.0
        set_request.state.pose.orientation.w = 1.0
        
        # Calculate and set velocity (tangent to the circle)
        set_request.state.twist.linear.x = linear_x
        set_request.state.twist.linear.y = linear_y
        set_request.state.twist.linear.z = linear_z
    
        return set_request
    
    def stop_ball(self):

        set_request = self.get_request(0.0, 0.0, self.z, 0.0, 0.0, 0.0)

        # Call the service synchronously to ensure it completes before shutdown
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        future = self.set_state_client.call_async(set_request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Ball stopped successfully')
        else:
            self.get_logger().error('Failed to stop ball')

    
def main(args=None):
    rclpy.init(args=args)
    ball_mover = BallMover()
    
    try:
        rclpy.spin(ball_mover)
    except KeyboardInterrupt:
        pass
    ball_mover.stop_ball()
    ball_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# from gazebo_msgs.srv import SetEntityState
# from gazebo_msgs.msg import EntityState
# import math
# import time

# class BallMover(Node):
#     def __init__(self):
#         super().__init__('ball_mover')
#         self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
#         self.timer = self.create_timer(0.5, self.move_ball)  # 20Hz update rate
#         self.angle = 0.0
#         self.radius = 1.0
#         self.center_x = 0.0
#         self.center_y = 0.0
#         self.z = 0.03  # Assuming ball radius is 3cm, to keep it just touching the ground

#     def move_ball(self):
#         set_request = SetEntityState.Request()
#         set_request.state = EntityState()
#         set_request.state.name = 'ball'
        
#         # Calculate new position
#         x = self.center_x + self.radius * math.cos(self.angle)
#         y = self.center_y + self.radius * math.sin(self.angle)
        
#         set_request.state.pose.position.x = x
#         set_request.state.pose.position.y = y
#         set_request.state.pose.position.z = self.z
        
#         # Set orientation (assuming the ball doesn't rotate)
#         set_request.state.pose.orientation.x = 0.0
#         set_request.state.pose.orientation.y = 0.0
#         set_request.state.pose.orientation.z = 0.0
#         set_request.state.pose.orientation.w = 1.0
        
#         # Calculate and set velocity (tangent to the circle)
#         velocity = 2.0  # meters per second, adjust as needed
#         set_request.state.twist.linear.x = -velocity * math.sin(self.angle)
#         set_request.state.twist.linear.y = velocity * math.cos(self.angle)
#         set_request.state.twist.linear.z = 0.0
        
#         self.future = self.set_state_client.call_async(set_request)
        
#         # Update angle for next iteration
#         self.angle += 0.05  # Adjust this value to change speed of rotation
#         if self.angle > 2 * math.pi:
#             self.angle -= 2 * math.pi

# def main(args=None):
#     rclpy.init(args=args)
#     ball_mover = BallMover()
    
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(ball_mover)
#             if ball_mover.future and ball_mover.future.done():
#                 try:
#                     response = ball_mover.future.result()
#                     if not response.success:
#                         ball_mover.get_logger().warn('Failed to move ball')
#                 except Exception as e:
#                     ball_mover.get_logger().error(f'Service call failed: {e}')
#                 ball_mover.future = None
#             time.sleep(0.01)
#     except KeyboardInterrupt:
#         pass
    
#     ball_mover.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from gazebo_msgs.srv import GetEntityState, SetEntityState
# from gazebo_msgs.msg import EntityState
# import time

# class BallMover(Node):
#     def __init__(self):
#         super().__init__('ball_mover')
#         self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
#         self.timer = self.create_timer(5.0, self.move_ball)  # Call every 5 seconds
#         self.future = None

#     def move_ball(self):
#         if self.future and not self.future.done():
#             self.get_logger().info('Still waiting for previous call to finish')
#             return

#         set_request = SetEntityState.Request()
#         set_request.state = EntityState()
#         set_request.state.name = 'ball'
#         set_request.state.pose.position.x = 0.0
#         set_request.state.pose.position.y = 1.0
#         set_request.state.pose.position.z = 0.5
#         set_request.state.pose.orientation.w = 1.0
        
#         self.get_logger().info(f'Sending request: {set_request}')
#         self.future = self.set_state_client.call_async(set_request)

# def main(args=None):
#     rclpy.init(args=args)
#     ball_mover = BallMover()
    
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(ball_mover)
#             if ball_mover.future and ball_mover.future.done():
#                 try:
#                     response = ball_mover.future.result()
#                     ball_mover.get_logger().info(f'Received response: {response}')
#                 except Exception as e:
#                     ball_mover.get_logger().error(f'Service call failed: {e}')
#                 ball_mover.future = None
#             time.sleep(0.1)
#     except KeyboardInterrupt:
#         pass
    
#     ball_mover.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()












# import rclpy
# from rclpy.node import Node
# from gazebo_msgs.srv import GetEntityState, SetEntityState
# from gazebo_msgs.msg import EntityState

# class BallMover(Node):
#     def __init__(self):
#         super().__init__('ball_mover')
#         self.get_state_client = self.create_client(GetEntityState, '/get_entity_state')
#         self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        
#         while not self.get_state_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('get_entity_state service not available, waiting again...')
        
#         while not self.set_state_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('set_entity_state service not available, waiting again...')
        
#         self.timer = self.create_timer(0.5, self.move_ball)

#     def move_ball(self):
#         self.get_logger().info('Attempting to move the ball...')
        
#         set_request = SetEntityState.Request()
#         set_request.state = EntityState()
#         set_request.state.name = 'ball'
#         set_request.state.pose.position.x = 1.0
#         set_request.state.pose.position.y = 0.0
#         set_request.state.pose.position.z = 0.5
#         set_request.state.pose.orientation.w = 1.0
        
#         self.get_logger().info(f'Sending request: {set_request}')
#         future = self.set_state_client.call_async(set_request)
#         rclpy.spin_until_future_complete(self, future)
        
#         if future.result() is not None:
#             response = future.result()
#             self.get_logger().info(f'Received response: {response}')
#             if response.success:
#                 self.get_logger().info('Ball move command sent successfully')
#             else:
#                 self.get_logger().error('Failed to move ball')
#         else:
#             self.get_logger().error('Service call failed')

# def main(args=None):
#     rclpy.init(args=args)
#     ball_mover = BallMover()
#     rclpy.spin(ball_mover)
#     ball_mover.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from gazebo_msgs.msg import ModelState
# from gazebo_msgs.srv import SetModelState

# class BallMover(Node):
#     def __init__(self):
#         super().__init__('ball_mover')
#         self.set_model_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
#         while not self.set_model_state_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.timer = self.create_timer(0.1, self.move_ball)

#     def move_ball(self):
#         request = SetModelState.Request()
#         request.model_state = ModelState()
#         request.model_state.model_name = 'ball'
#         request.model_state.pose.position.x = 1.0  # Move to x=1
#         request.model_state.pose.position.y = 0.0
#         request.model_state.pose.position.z = 0.03
        
#         future = self.set_model_state_client.call_async(request)
#         rclpy.spin_until_future_complete(self, future)
        
#         if future.result() is not None:
#             self.get_logger().info('Ball moved successfully')
#         else:
#             self.get_logger().error('Failed to move ball')

# def main(args=None):
#     rclpy.init(args=args)
#     ball_mover = BallMover()
#     rclpy.spin(ball_mover)
#     ball_mover.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()