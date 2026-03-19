#!/usr/bin/env python3

import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class ClosedLoopSquare:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        
        # Encoder tick counts
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_ticks_start = 0
        self.right_ticks_start = 0
        
        # Robot parameters
        self.ticks_per_meter = 565  # calibrate this
        self.ticks_per_90_degrees = 195  # calibrate this
        
        # State
        self.moving = False
        self.target_ticks = 0
        self.current_action = None
        
        rospy.init_node('closed_loop_square_node', anonymous=True)
        
        self.pub = rospy.Publisher('/vehicle_0/car_cmd_switch_node/cmd', 
                                    Twist2DStamped, queue_size=1)
        rospy.Subscriber('/vehicle_0/left_wheel_encoder_node/tick', 
                         WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/vehicle_0/right_wheel_encoder_node/tick', 
                         WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber('/vehicle_0/fsm_node/mode', 
                         FSMState, self.fsm_callback)
        
        rospy.loginfo("Closed loop node initialized!")

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data
        self.check_goal()

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data
        self.check_goal()

    def check_goal(self):
        if not self.moving:
            return
        avg_ticks = abs(((self.left_ticks - self.left_ticks_start) + 
                         (self.right_ticks - self.right_ticks_start)) / 2)
        if avg_ticks >= self.target_ticks:
            self.stop_robot()
            self.moving = False
            rospy.loginfo("Goal reached! Ticks: %d", avg_ticks)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def move_straight(self, distance, speed=0.5):
        # distance in meters
        self.target_ticks = abs(distance) * self.ticks_per_meter
        self.left_ticks_start = self.left_ticks
        self.right_ticks_start = self.right_ticks
        self.moving = True
        
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = speed if distance > 0 else -speed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Moving straight: %.2fm at speed %.2f", distance, speed)
        
        while self.moving and not rospy.is_shutdown():
            rospy.sleep(0.01)

    def rotate(self, degrees, speed=2.0):
        # degrees: positive=left, negative=right
        self.target_ticks = abs(degrees / 90.0) * self.ticks_per_90_degrees
        self.left_ticks_start = self.left_ticks
        self.right_ticks_start = self.right_ticks
        self.moving = True
        
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = speed if degrees > 0 else -speed
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Rotating: %d degrees at speed %.2f", degrees, speed)
        
        while self.moving and not rospy.is_shutdown():
            rospy.sleep(0.01)

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)
            self.run_square()

    def run_square(self):
        rospy.loginfo("Starting closed loop square!")
        for i in range(4):
            rospy.loginfo("Side %d", i+1)
            self.move_straight(1.0, speed=0.5)
            rospy.sleep(0.5)
            self.rotate(90, speed=2.0)
            rospy.sleep(0.5)
        self.stop_robot()
        rospy.loginfo("Square complete!")

if __name__ == '__main__':
    try:
        node = ClosedLoopSquare()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
