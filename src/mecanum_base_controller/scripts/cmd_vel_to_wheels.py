#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class CmdVelToWheelsDifferential:
    def __init__(self):
        rospy.init_node("cmd_vel_to_wheels_diff")

        # ===== Robot parameters =====
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.04)  # meters
        self.robot_width  = rospy.get_param("~robot_width",  0.30)  # distance between left and right wheels

        # ===== ROS =====
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.wheel_pub = rospy.Publisher(
            "/wheel_speeds",
            Float32MultiArray,
            queue_size=10
        )

        rospy.loginfo("cmd_vel → differential 4-wheel speed node started")

    # ============================================================
    def cmd_vel_callback(self, msg):
        vx = msg.linear.x  # forward velocity
        wz = msg.angular.z # rotational velocity around z

        # Differential drive inverse kinematics
        # v_left  = vx - (wz * wheel_base / 2)
        # v_right = vx + (wz * wheel_base / 2)
        v_left  = vx - (wz * self.robot_width / 2.0)
        v_right = vx + (wz * self.robot_width / 2.0)

        # convert linear velocity to wheel angular velocity (rad/s)
        w_left  = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius

        # 4 wheels: [fl, fr, rl, rr]
        wheel_msg = Float32MultiArray()
        wheel_msg.data = [w_left, w_right, w_left, w_right]

        self.wheel_pub.publish(wheel_msg)


if __name__ == "__main__":
    try:
        CmdVelToWheelsDifferential()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
