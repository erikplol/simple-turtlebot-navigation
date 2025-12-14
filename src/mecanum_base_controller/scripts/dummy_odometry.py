#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import tf

def main():
    rospy.init_node("dummy_odometry")
    pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10 Hz

    x = 0.0
    y = 0.0
    yaw = 0.0
    vx = 0.0  # m/s
    vy = 0.0
    wz = 0.0  # rad/s

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        # integrate pose
        x += vx * 0.1
        y += vy * 0.1
        yaw += wz * 0.1

        # quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

        # publish TF
        br.sendTransform(
            (x, y, 0),
            quat,
            now,
            "base_link",
            "odom"
        )

        # publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        pub.publish(odom)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
