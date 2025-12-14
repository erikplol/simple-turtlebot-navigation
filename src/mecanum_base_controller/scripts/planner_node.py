#!/usr/bin/env python3

import rospy
import math
import heapq
import tf
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import tf.transformations as tf_trans

class PlannerNode:
    def __init__(self):
        rospy.init_node('planner_node', anonymous=True)
        
        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 1.0)
        self.angular_speed = rospy.get_param('~angular_speed', 2.0)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.05)
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.15)
        self.obstacle_distance = rospy.get_param('~obstacle_distance', 0.2)
        self.scan_angle_range = rospy.get_param('~scan_angle_range', 90.0)  # degrees
        self.detour_speed = rospy.get_param('~detour_speed', 0.1)
        self.clear_path_threshold = rospy.get_param('~clear_path_threshold', 0.3)
        self.padding_radius = rospy.get_param('~padding_radius', 0.05)  # meters, keep objects outside robot safety bubble
        self.stuck_timeout = rospy.get_param('~stuck_timeout', 15.0)  # seconds without progress before failing goal
        self.progress_epsilon = rospy.get_param('~progress_epsilon', 0.05)  # meters of progress considered meaningful
        self.costmap_cells = rospy.get_param('~costmap_cells', 36)  # number of angular cells in local polar costmap
        self.costmap_radius = rospy.get_param('~costmap_radius', 3.0)  # meters to consider for local costmap
        self.costmap_inflation = rospy.get_param('~costmap_inflation', 0.05)  # extra inflation beyond padding
        self.costmap_resolution = rospy.get_param('~costmap_resolution', 0.1)  # grid resolution for A* local planner
        self.path_consistency_penalty = rospy.get_param('~path_consistency_penalty', 0.5)  # cost penalty for deviating from last path
        
        # State variables
        self.current_goal = None
        self.has_goal = False
        self.last_path_direction = None  # track preferred direction (left/right/straight)
        self.scan_data = None
        self.robot_pose = None
        self.last_progress_time = rospy.Time.now()
        self.last_distance = None
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)  # Give TF listener time to buffer transforms
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        
        # Subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
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
        safety_distance = self.obstacle_distance + self.padding_radius
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            
            # Check if angle is within frontal cone
            if abs(angle) <= angle_range:
                if distance < safety_distance and distance > self.scan_data.range_min:
                    obstacle_detected = True
                    break
        
        return obstacle_detected
    
    def path_clear_ahead(self):
        """Check if forward sector has enough clearance to proceed"""
        if self.scan_data is None:
            return False
        ranges = list(self.scan_data.ranges)
        n = len(ranges)
        s = n // 5
        front = ranges[2*s:3*s]
        front_valid = [r for r in front if self.scan_data.range_min < r < self.scan_data.range_max]
        if not front_valid:
            return False
        avg_front = sum(front_valid) / len(front_valid)
        # Require clearance ahead accounting for padding
        required_clearance = max(self.clear_path_threshold, self.obstacle_distance + self.padding_radius)
        return avg_front > required_clearance
    
    def calculate_distance_to_goal(self, goal_x, goal_y, current_x, current_y):
        """Calculate Euclidean distance to goal"""
        return math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
    
    def calculate_angle_to_goal(self, goal_x, goal_y, current_x, current_y):
        """Calculate angle to goal relative to current position"""
        return math.atan2(goal_y - current_y, goal_x - current_x)
    
    def get_robot_pose(self, target_frame='map'):
        """Get robot pose from TF"""
        try:
            # Lookup transform from target frame to base_link
            (trans, rot) = self.tf_listener.lookupTransform(target_frame, 'base_link', rospy.Time(0))
            
            # Extract position
            x = trans[0]
            y = trans[1]
            
            # Extract yaw from quaternion
            yaw = self.get_yaw_from_quaternion(type('obj', (object,), {
                'x': rot[0], 'y': rot[1], 'z': rot[2], 'w': rot[3]
            })())
            
            # Cache for costmap goal heading
            self.robot_pose = (x, y, yaw)
            return x, y, yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF lookup failed: {e}")
            return None, None, None

    def publish_path(self, frame_id, poses):
        """Publish a Path message given a list of PoseStamped"""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame_id
        path.poses = poses
        self.path_pub.publish(path)

    def build_local_grid(self):
        """Build a local occupancy grid in the robot frame from LaserScan"""
        if self.scan_data is None:
            return None, None, None

        res = self.costmap_resolution
        radius = self.costmap_radius
        size = int(2 * radius / res) + 1
        grid = [[0 for _ in range(size)] for _ in range(size)]
        origin_x = -radius
        origin_y = -radius

        inflation = self.obstacle_distance + self.padding_radius + self.costmap_inflation
        inflation_cells = int(math.ceil(inflation / res))

        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment

        def mark_cell(cx, cy):
            if 0 <= cx < size and 0 <= cy < size:
                grid[cy][cx] = 1

        for i, dist in enumerate(self.scan_data.ranges):
            if dist < self.scan_data.range_min or dist > self.scan_data.range_max:
                continue
            if dist > radius:
                continue
            ang = angle_min + i * angle_inc
            x = dist * math.cos(ang)
            y = dist * math.sin(ang)
            gx = int((x - origin_x) / res)
            gy = int((y - origin_y) / res)
            if 0 <= gx < size and 0 <= gy < size:
                # Inflate obstacle area
                for dx in range(-inflation_cells, inflation_cells + 1):
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        if dx * dx + dy * dy <= inflation_cells * inflation_cells:
                            mark_cell(gx + dx, gy + dy)

        return grid, origin_x, origin_y

    def astar(self, grid, start, goal):
        """Run A* on the occupancy grid"""
        if grid is None:
            return None
        h = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1])
        open_set = []
        heapq.heappush(open_set, (0 + h(start, goal), 0, start, None))
        came_from = {}
        g_score = {start: 0}
        visited = set()
        moves = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))
        ]
        max_y = len(grid)
        max_x = len(grid[0]) if max_y > 0 else 0

        while open_set:
            f, g, node, parent = heapq.heappop(open_set)
            if node in visited:
                continue
            visited.add(node)
            came_from[node] = parent
            if node == goal:
                # Reconstruct
                path = []
                cur = node
                while cur:
                    path.append(cur)
                    cur = came_from[cur]
                path.reverse()
                return path
            for dx, dy, cost in moves:
                nx, ny = node[0] + dx, node[1] + dy
                if nx < 0 or ny < 0 or nx >= max_x or ny >= max_y:
                    continue
                if grid[ny][nx] == 1:
                    continue
                new_g = g + cost
                # Apply consistency penalty: penalize moves opposite to last direction
                if self.last_path_direction is not None:
                    if self.last_path_direction == 'left' and dx > 0:
                        new_g += self.path_consistency_penalty
                    elif self.last_path_direction == 'right' and dx < 0:
                        new_g += self.path_consistency_penalty
                if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_g
                    f_score = new_g + h((nx, ny), goal)
                    heapq.heappush(open_set, (f_score, new_g, (nx, ny), node))
        return None

    def plan_local_path(self, goal_frame, current_pose, goal_pose):
        """Plan a local path using A* on a robot-centric grid."""
        grid, origin_x, origin_y = self.build_local_grid()
        if grid is None or current_pose is None:
            return None

        cx, cy, cyaw = current_pose

        # Goal in world frame (goal_frame == map/odom). Convert to robot frame.
        gx_world = goal_pose.pose.position.x
        gy_world = goal_pose.pose.position.y
        dx = gx_world - cx
        dy = gy_world - cy
        lx = math.cos(cyaw) * dx + math.sin(cyaw) * dy
        ly = -math.sin(cyaw) * dx + math.cos(cyaw) * dy

        goal_dist = math.hypot(lx, ly)
        if goal_dist > self.costmap_radius * 0.9:
            return None

        res = self.costmap_resolution
        size = len(grid)
        start_cell = (int((-origin_x) / res), int((-origin_y) / res))
        goal_cell = (int((lx - origin_x) / res), int((ly - origin_y) / res))

        if not (0 <= goal_cell[0] < size and 0 <= goal_cell[1] < size):
            return None

        path_cells = self.astar(grid, start_cell, goal_cell)
        if not path_cells:
            return None
        
        # Determine direction and update preference for consistency
        if len(path_cells) > 1:
            dx_path = path_cells[1][0] - path_cells[0][0]
            if dx_path < -0.5:
                self.last_path_direction = 'left'
            elif dx_path > 0.5:
                self.last_path_direction = 'right'
            else:
                self.last_path_direction = 'straight'

        poses = []
        frame_id = goal_frame
        for cell in path_cells:
            px_local = origin_x + (cell[0] + 0.5) * res
            py_local = origin_y + (cell[1] + 0.5) * res
            # Transform back to world frame
            wx = cx + math.cos(cyaw) * px_local - math.sin(cyaw) * py_local
            wy = cy + math.sin(cyaw) * px_local + math.cos(cyaw) * py_local
            p = PoseStamped()
            p.header.frame_id = frame_id
            p.header.stamp = rospy.Time.now()
            p.pose.position.x = wx
            p.pose.position.y = wy
            p.pose.position.z = 0.0
            q = tf_trans.quaternion_from_euler(0, 0, cyaw)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            poses.append(p)

        return poses
    
    def control_loop(self):
        """Main control loop"""
        while not rospy.is_shutdown():
            if self.has_goal and self.current_goal is not None:
                # Get goal position
                goal_x = self.current_goal.pose.position.x
                goal_y = self.current_goal.pose.position.y
                
                # Get current robot pose from TF
                goal_frame = self.current_goal.header.frame_id if self.current_goal.header.frame_id else 'map'
                current_x, current_y, current_yaw = self.get_robot_pose(goal_frame)
                
                # Skip if TF lookup failed
                if current_x is None:
                    self.control_rate.sleep()
                    continue

                # Plan local path with A*
                path_poses = self.plan_local_path(goal_frame, (current_x, current_y, current_yaw), self.current_goal)
                if path_poses:
                    self.publish_path(goal_frame, path_poses)
                
                # Calculate distance and angle to goal
                distance_to_goal = self.calculate_distance_to_goal(goal_x, goal_y, current_x, current_y)
                angle_to_goal = self.calculate_angle_to_goal(goal_x, goal_y, current_x, current_y)
                angle_diff = angle_to_goal - current_yaw
                
                # Normalize angle difference to [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # Log current state
                rospy.loginfo_throttle(1.0, f"Current: ({current_x:.2f}, {current_y:.2f}, yaw={math.degrees(current_yaw):.1f}°) | "
                                           f"Goal: ({goal_x:.2f}, {goal_y:.2f}) | "
                                           f"Distance: {distance_to_goal:.2f}m | "
                                           f"Angle diff: {math.degrees(angle_diff):.1f}°")
                
                # Create velocity command
                cmd = Twist()
                
                # Check if goal is reached
                if distance_to_goal < self.goal_tolerance:
                    rospy.loginfo("Goal reached!")
                    self.has_goal = False
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.last_distance = None
                    self.cmd_vel_pub.publish(cmd)
                    continue

                # Track progress to detect being stuck
                if self.last_distance is None:
                    self.last_distance = distance_to_goal
                    self.last_progress_time = rospy.Time.now()
                else:
                    if distance_to_goal < self.last_distance - self.progress_epsilon:
                        self.last_progress_time = rospy.Time.now()
                        self.last_distance = distance_to_goal
                    # If no progress for too long, fail the goal
                    time_since_progress = (rospy.Time.now() - self.last_progress_time).to_sec()
                    if time_since_progress > self.stuck_timeout:
                        rospy.logwarn("Goal failed: stuck with no progress")
                        self.has_goal = False
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd)
                        self.last_distance = None
                        continue
                
                # Check for obstacles (safety only) and plan availability
                obstacle_ahead = self.check_obstacle_in_front()
                has_path = path_poses is not None and len(path_poses) > 1

                # Select local waypoint from planned path
                if has_path:
                    wp = path_poses[min(2, len(path_poses) - 1)]  # skip start, take next step
                    wp_x = wp.pose.position.x
                    wp_y = wp.pose.position.y
                    dist_wp = self.calculate_distance_to_goal(wp_x, wp_y, current_x, current_y)
                    ang_wp = self.calculate_angle_to_goal(wp_x, wp_y, current_x, current_y)
                    ang_diff_wp = ang_wp - current_yaw
                    while ang_diff_wp > math.pi:
                        ang_diff_wp -= 2 * math.pi
                    while ang_diff_wp < -math.pi:
                        ang_diff_wp += 2 * math.pi
                else:
                    dist_wp = None
                    ang_diff_wp = None

                # Navigation policy: path-following only; stop if no path or unsafe
                if has_path:
                    if abs(ang_diff_wp) > self.angle_tolerance:
                        cmd.linear.x = 0.0
                        cmd.angular.z = self.angular_speed if ang_diff_wp > 0 else -self.angular_speed
                    else:
                        cmd.linear.x = self.linear_speed
                        cmd.angular.z = 0.0
                    if obstacle_ahead:
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0
                else:
                    rospy.logwarn_throttle(2.0, "No valid path; holding position")
                    cmd.linear.x = 0.0
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
