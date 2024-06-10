# Ball tracking based on this posts 
# 1.  https://pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
# 2. https://www.instructables.com/Raspberry-Pi-Ball-tracking/


import cv2 as cv
from imutils.video import VideoStream
import numpy as np
import argparse
import time
import imutils # to insatll: $ pip install --upgrade imutils



# Known diameter of the tennis ball in cm
KNOWN_DIAMETER = 6.5
# Focal length of the camera (to be calibrated for accurate distance measurement) in px
FOCAL_LENGTH = 570

# Define HSV range for yellow color of the tennis ball
yellowLowerO = (20, 100, 100)
yellowUpperO = (45, 185, 255)

yellowLowerYo = (38, 86, 100)
yellowUpperYo = (89, 110, 255)

yellowLowerYu = (36, 64, 156)
yellowUpperYu = (54, 255, 255)

yellowLower = [yellowLowerO, yellowLowerYo, yellowLowerYu]
yellowUpper = [yellowUpperO, yellowUpperYo, yellowUpperYu]

indexYellow = 2

prev_circle = None
dist = lambda x1, y1, x2, y2: (x1-x2)**2+(y1-y2)**2

green = (0,255,0)
red = (0,0,255)
blue = (255,0,0)
yellow = (0, 255, 255)
purple = (128,0,128)



def nothing(x):
    pass

blur_Ksize_color=11
iteration_color=2

blur_Ksize_shape=5
iteration_shape=1
dp=2
minDist=120
param1=120
param2=50
minRadius=10
maxRadius=0

def calculate_distance(diameter_px, known_diameter=KNOWN_DIAMETER, focal_length=FOCAL_LENGTH):
    """
    Calculate the distance to an object using its diameter in pixels.

    :param diameter_px: Diameter of the detected object in pixels.
    :param known_diameter: Known diameter of the object in cm.
    :param focal_length: Focal length of the camera in pixels.
    :return: Estimated distance to the object in cm.
    """
    return (known_diameter * focal_length) / diameter_px



def draw_circles_and_info(frame, center, radius, distance):
    
    radius = int(radius)
    cv.putText(frame, f"Radiue: {radius} px", (10, frame.shape[0] - 50), cv.FONT_HERSHEY_SIMPLEX, 0.65, blue, 2)
    cv.putText(frame, f"center(x, y) : {center} px", (10, frame.shape[0] - 30), cv.FONT_HERSHEY_SIMPLEX, 0.65, blue, 2)
    cv.putText(frame, f"Distance: {distance:.2f} cm", (10, frame.shape[0] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.65, blue, 2)
    
    if radius > 10:            
        cv.circle(frame, center, radius, green, 3)  # Draw the outer circle
        cv.circle(frame, center, 3, red, -1)        # Draw the center of the circle


def main(): 
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
    args = vars(ap.parse_args())

    if not args.get("video", False):
        vs = VideoStream(src=0).start()
    else:
        vs = cv.VideoCapture(args["video"])
    
    # allow the camera or video file to warm up
    time.sleep(2.0)

    while True:

        frame = vs.read() # grab the current frame
        frame = frame[1] if args.get("video", False) else frame # handle the frame from VideoCapture or VideoStream
        
        # if we are viewing a video and we did not grab a frame, then we have reached the end of the video
        if frame is None:
            break
        
        # resize the frame, blur it, and convert it to the HSV color space

        frame = imutils.resize(frame, width=600)
        frame_for_shape = frame.copy()

        detect_ball, center, radius, distance = find_cnts_by_color(frame)
        if detect_ball:
            draw_circles_and_info(frame, center, radius, distance)
        

        detect_ball, center, radius, distance, mask_of_circles = find_cnts_by_color_and_shape(frame_for_shape)  
        if detect_ball:
            draw_circles_and_info(frame_for_shape, center, radius, distance)



        cv.imshow("Frame", frame)
        cv.imshow("mask_of_circles", mask_of_circles)
        
        cv.imshow("frame_for_shape", frame_for_shape)

        if cv.waitKey(1) & 0xFF == ord("q"): # if the 'q' key is pressed, stop the loop
            break
    
    
    if not args.get("video", False):
        vs.stop()
    else:
        vs.release()
    
    # close all windows
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()

###############################################################



# Copyright 2023 Josh Newans
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time

class FollowBall(Node):

    def __init__(self):
        super().__init__('follow_ball')
        self.subscription = self.create_subscription(Point, '/detected_ball', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)


        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.7)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)


        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value


        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

    def timer_callback(self):
        msg = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            self.get_logger().info('Target: {}'.format(self.target_val))
            print(self.target_dist)
            if (self.target_dist < self.max_size_thresh):
                msg.linear.x = self.forward_chase_speed
            msg.angular.z = -self.angular_chase_multiplier*self.target_val
        else:
            self.get_logger().info('Target lost')
            msg.angular.z = self.search_angular_speed
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        f = self.filter_value # 0.9
        self.target_val = self.target_val * f + msg.x * (1-f)
        self.target_dist = self.target_dist * f + msg.z * (1-f)
        self.lastrcvtime = time.time()
        # self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)
    follow_ball = FollowBall()
    rclpy.spin(follow_ball)
    follow_ball.destroy_node()
    rclpy.shutdown()