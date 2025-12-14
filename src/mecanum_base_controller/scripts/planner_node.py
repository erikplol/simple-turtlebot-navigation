#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import tf.transformations as tf_trans

class PlannerNode:
    def __init__(self):
        rospy.init_node('planner_node', anonymous=True)
        
        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.1)
        self.obstacle_distance = rospy.get_param('~obstacle_distance', 0.5)
        self.scan_angle_range = rospy.get_param('~scan_angle_range', 30.0)  # degrees
        
        # State variables
        self.current_goal = None
        self.has_goal = False
        self.scan_data = None
        self.robot_pose = None
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        self.goal_sub = rospy.Subscriber('/goal', PoseStamped, self.goal_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Timer for control loop
        self.control_rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Planner node initialized")
    
    def goal_callback(self, msg):
        """Callback for receiving goal pose"""
        self.current_goal = msg
        self.has_goal = True
        rospy.loginfo(f"New goal received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
    
    def scan_callback(self, msg):
        """Callback for receiving laser scan data"""
        self.scan_data = msg
    
    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw angle from quaternion"""
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = tf_trans.euler_from_quaternion(orientation_list)
        return yaw
    
    def check_obstacle_in_front(self):
        """Check if there's an obstacle in front of the robot"""
        if self.scan_data is None:
            return False
        
        # Convert scan angle range from degrees to radians
        angle_range = math.radians(self.scan_angle_range)
        
        # Check frontal cone
        ranges = self.scan_data.ranges
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        
        obstacle_detected = False
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            
            # Check if angle is within frontal cone
            if abs(angle) <= angle_range:
                if distance < self.obstacle_distance and distance > self.scan_data.range_min:
                    obstacle_detected = True
                    break
        
        return obstacle_detected
    
    def find_clear_direction(self):
        """Find the clearest direction to move"""
        if self.scan_data is None:
            return 0.0
        
        # Divide scan into left and right sectors
        ranges = self.scan_data.ranges
        mid_idx = len(ranges) // 2
        
        # Calculate average distance in left and right sectors
        left_distances = [r for r in ranges[:mid_idx] if self.scan_data.range_min < r < self.scan_data.range_max]
        right_distances = [r for r in ranges[mid_idx:] if self.scan_data.range_min < r < self.scan_data.range_max]
        
        avg_left = sum(left_distances) / len(left_distances) if left_distances else 0.0
        avg_right = sum(right_distances) / len(right_distances) if right_distances else 0.0
        
        # Return angular velocity to steer toward clearer side
        if avg_left > avg_right:
            return self.angular_speed  # Turn left
        else:
            return -self.angular_speed  # Turn right
    
    def calculate_distance_to_goal(self, goal_x, goal_y, current_x, current_y):
        """Calculate Euclidean distance to goal"""
        return math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
    
    def calculate_angle_to_goal(self, goal_x, goal_y, current_x, current_y):
        """Calculate angle to goal relative to current position"""
        return math.atan2(goal_y - current_y, goal_x - current_x)
    
    def control_loop(self):
        """Main control loop"""
        while not rospy.is_shutdown():
            if self.has_goal and self.current_goal is not None:
                # Get goal position
                goal_x = self.current_goal.pose.position.x
                goal_y = self.current_goal.pose.position.y
                
                # For straight-line navigation, assume robot starts at origin
                # In a real implementation, you would use tf to get robot pose
                current_x = 0.0
                current_y = 0.0
                current_yaw = 0.0
                
                # Calculate distance and angle to goal
                distance_to_goal = self.calculate_distance_to_goal(goal_x, goal_y, current_x, current_y)
                angle_to_goal = self.calculate_angle_to_goal(goal_x, goal_y, current_x, current_y)
                angle_diff = angle_to_goal - current_yaw
                
                # Normalize angle difference to [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # Create velocity command
                cmd = Twist()
                
                # Check if goal is reached
                if distance_to_goal < self.goal_tolerance:
                    rospy.loginfo("Goal reached!")
                    self.has_goal = False
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd)
                    continue
                
                # Check for obstacles
                obstacle_ahead = self.check_obstacle_in_front()
                
                if obstacle_ahead:
                    # Obstacle avoidance: stop forward motion and turn
                    rospy.logwarn("Obstacle detected! Avoiding...")
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.find_clear_direction()
                else:
                    # Navigate toward goal
                    if abs(angle_diff) > self.angle_tolerance:
                        # Rotate toward goal
                        cmd.linear.x = 0.1  # Slow forward motion while rotating
                        cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                    else:
                        # Move straight toward goal
                        cmd.linear.x = self.linear_speed
                        cmd.angular.z = 0.0
                
                # Publish command
                self.cmd_vel_pub.publish(cmd)
            else:
                # No goal, publish zero velocity
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
            
            self.control_rate.sleep()
    
    def run(self):
        """Start the planner node"""
        try:
            self.control_loop()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        planner = PlannerNode()
        planner.run()
    except rospy.ROSInterruptException:
        pass
