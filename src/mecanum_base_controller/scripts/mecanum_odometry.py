#!/usr/bin/env python3

import rospy
import math
import tf
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float32MultiArray


class MecanumOdometry:
    def __init__(self):
        rospy.init_node("mecanum_odometry")

        # ===== Robot parameters =====
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)   # meters
        self.robot_length = rospy.get_param("~robot_length", 0.30)   # meters
        self.robot_width  = rospy.get_param("~robot_width",  0.30)   # meters
        self.ticks_per_rev = rospy.get_param("~ticks_per_rev", 2048)

        self.k = (self.robot_length / 2.0) + (self.robot_width / 2.0)

        # ===== State =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = rospy.Time.now()

        self.prev_ticks = [0, 0, 0, 0]

        # ===== ROS =====
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/wheel_ticks", Float32MultiArray, self.encoder_callback)

        rospy.loginfo("Mecanum odometry node started")

    # ============================================================
    def encoder_callback(self, msg):
        """
        msg.data = [fl, fr, rl, rr] encoder ticks
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        if dt <= 0.0:
            return

        ticks = msg.data
        d_ticks = [ticks[i] - self.prev_ticks[i] for i in range(4)]
        self.prev_ticks = ticks

        # ticks → radians
        d_theta = [(2.0 * math.pi * dtick) / self.ticks_per_rev for dtick in d_ticks]

        # wheel linear velocity
        w = [self.wheel_radius * th / dt for th in d_theta]

        # ===== Mecanum forward kinematics =====
        vx = (w[0] + w[1] + w[2] + w[3]) / 4.0
        vy = (-w[0] + w[1] + w[2] - w[3]) / 4.0
        wz = (-w[0] + w[1] - w[2] + w[3]) / (4.0 * self.k)

        # ===== Integrate pose =====
        dx = vx * math.cos(self.yaw) - vy * math.sin(self.yaw)
        dy = vx * math.sin(self.yaw) + vy * math.cos(self.yaw)

        self.x += dx * dt
        self.y += dy * dt
        self.yaw += wz * dt

        self.publish_odometry(vx, vy, wz, current_time)

        self.last_time = current_time

    # ============================================================
    def publish_odometry(self, vx, vy, wz, time):
        quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)

        # TF
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            quat,
            time,
            "base_link",
            "odom"
        )

        # Odometry message
        odom = Odometry()
        odom.header.stamp = time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)


if __name__ == "__main__":
    try:
        MecanumOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

