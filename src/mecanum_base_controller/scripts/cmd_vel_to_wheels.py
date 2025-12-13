#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class CmdVelToWheels:
    def __init__(self):
        rospy.init_node("cmd_vel_to_wheels")

        # ===== Robot parameters =====
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)  # meters
        self.robot_length = rospy.get_param("~robot_length", 0.30)  # meters
        self.robot_width  = rospy.get_param("~robot_width",  0.30)  # meters

        self.k = (self.robot_length / 2.0) + (self.robot_width / 2.0)

        # ===== ROS =====
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.wheel_pub = rospy.Publisher(
            "/wheel_speeds",
            Float32MultiArray,
            queue_size=10
        )

        rospy.loginfo("cmd_vel → mecanum wheel speed node started")

    # ============================================================
    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Inverse kinematics
        w_fl = (vx - vy - self.k * wz) / self.wheel_radius
        w_fr = (vx + vy + self.k * wz) / self.wheel_radius
        w_rl = (vx + vy - self.k * wz) / self.wheel_radius
        w_rr = (vx - vy + self.k * wz) / self.wheel_radius

        wheel_msg = Float32MultiArray()
        wheel_msg.data = [w_fl, w_fr, w_rl, w_rr]

        self.wheel_pub.publish(wheel_msg)


if __name__ == "__main__":
    try:
        CmdVelToWheels()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
