#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        # Parameters
        self.omega_seek = 1.5      # Rotation speed when seeking
        self.omega_max = 3.0       # Max rotation speed when tracking
        self.omega_min = 0.5       # Min rotation speed to overcome friction
        self.x_threshold = 0.05   # Dead zone - if tag is within this, don't rotate
        self.tag_detected = False

        # Publisher and Subscriber
        self.cmd_vel_pub = rospy.Publisher('/vehicle_0/car_cmd_switch_node/cmd',
                                           Twist2DStamped, queue_size=1)
        rospy.Subscriber('/vehicle_0/apriltag_detector_node/detections',
                         AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.loginfo("Target follower node initialized!")
        rospy.spin()

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("Shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0

        if len(detections) == 0:
            # SEEK behavior - rotate to find a tag
            rospy.loginfo("No tag detected - SEEKING...")
            cmd_msg.omega = self.omega_seek
            self.tag_detected = False
        else:
            # LOOK AT behavior - keep tag centered
            x = detections[0].transform.translation.x
            y = detections[0].transform.translation.y
            z = detections[0].transform.translation.z
            rospy.loginfo("Tag detected! x=%.3f y=%.3f z=%.3f", x, y, z)

            # x is left/right offset - use proportional control
            error = x
            if abs(error) < self.x_threshold:
                # Tag is centered - stop rotating
                cmd_msg.omega = 0.0
                rospy.loginfo("Tag centered!")
            else:
                # Proportional control
                omega = -error * 5.0
                # Apply min/max limits
                if abs(omega) < self.omega_min:
                    omega = self.omega_min if omega > 0 else -self.omega_min
                if abs(omega) > self.omega_max:
                    omega = self.omega_max if omega > 0 else -self.omega_max
                cmd_msg.omega = omega
                rospy.loginfo("Tracking tag - omega=%.3f", omega)

            self.tag_detected = True

        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
