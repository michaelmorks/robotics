#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class DriveSquare:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/vehicle_0/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/vehicle_0/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)
            self.move_robot()

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def move_robot(self):
        # Complete one full lap around the loop map
        for _ in range(4):
            # Drive straight
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.5
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Forward!")
            rospy.sleep(2)

            # Turn 90 degrees
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 2.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Turning!")
            rospy.sleep(0.8)

        self.stop_robot()
        rospy.loginfo("Done!")

if __name__ == '__main__':
    try:
        node = DriveSquare()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
