import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import heapq

class TurtleAStar(Node):
    def __init__(self):
        super().__init__('turtle_astar')
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.turtle_pose = None
        self.targets = [(8.0, 8.0), (4.0, 8.0), (8.0, 4.0)]
        self.current_target_index = 0
        self.obstacles = []
        self.rate = self.create_rate(10)
    
    def pose_callback(self, msg):
        self.turtle_pose = (msg.x, msg.y, msg.theta)  
        if self.turtle_pose and self.current_target_index < len(self.targets):
            self.navigate_to_target()
    
    def navigate_to_target(self):
        current_target = self.targets[self.current_target_index]
        if self.reached_target(current_target):
            self.mark_obstacle(current_target)
            self.current_target_index += 1
            if self.current_target_index < len(self.targets):
                new_target_index = self.current_target_index
                initial_pose = self.turtle_pose
                
                while new_target_index < len(self.targets):
                    target_pose = self.targets[new_target_index]
                    
                    # Get the equation of the line from initial to current target
                    m1, b1 = self.build_eq(initial_pose[0], target_pose[0], initial_pose[1], target_pose[1])
                    
                    # Check for intersection with the next target
                    if new_target_index + 1 < len(self.targets):
                        next_target_pose = self.targets[new_target_index + 1]
                        m2, b2 = self.build_eq(target_pose[0], next_target_pose[0], target_pose[1], next_target_pose[1])
                        
                        if self.intersection_checker(m1, b1, m2, b2, initial_pose, target_pose):
                            # Adjust next target point to avoid intersection
                            if next_target_pose[0] > target_pose[0]:
                                next_target_pose = (next_target_pose[0] + 1, m2 * (next_target_pose[0] + 1) + b2)
                            else:
                                next_target_pose = (next_target_pose[0] - 1, m2 * (next_target_pose[0] - 1) + b2)

                            # Update the list of targets with adjusted next target
                            self.targets[new_target_index + 1] = next_target_pose
                    new_target_index += 1
        elif self.current_target_index < len(self.targets):
            path = self.astar(self.turtle_pose, current_target)
            if path:
                self.follow_path(path)
    
    def reached_target(self, target):
        if self.turtle_pose:
            distance = math.sqrt((target[0] - self.turtle_pose[0])**2 + (target[1] - self.turtle_pose[1])**2)
            return distance < 0.2  
        return False
    
    def mark_obstacle(self, target):
        if self.turtle_pose:
            self.obstacles.append((self.turtle_pose, target)) 
    
    def astar(self, start, goal):
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if self.reached_target(current):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
            
            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + heuristic(current, neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current
        return []
    
    def get_neighbors(self, current):
        step = 0.1
        neighbors = [
            (current[0] + step, current[1]),
            (current[0] - step, current[1]),
            (current[0], current[1] + step),
            (current[0], current[1] - step)
        ]
        
        valid_neighbors = []
        for neighbor in neighbors:
            if not self.is_collision(current, neighbor):
                valid_neighbors.append(neighbor)
        return valid_neighbors
    
    def is_collision(self, start, end):
        for obs_start, obs_end in self.obstacles:
            if self.do_lines_intersect(start, end, obs_start, obs_end):
                return True
        return False
    
    def do_lines_intersect(self, p1, q1, p2, q2):
        def orientation(p, q, r):
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0  
            elif val > 0:
                return 1  
            else:
                return 2  

        def on_segment(p, q, r):
            return min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])

        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        if o1 != o2 and o3 != o4:
            return True

        if o1 == 0 and on_segment(p1, p2, q1):
            return True
        if o2 == 0 and on_segment(p1, q2, q1):
            return True
        if o3 == 0 and on_segment(p2, p1, q2):
            return True
        if o4 == 0 and on_segment(p2, q1, q2):
            return True

        return False

    def build_eq(self, x1, x2, y1, y2):
        m = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
        b = y1 - m * x1
        return m, b

    def intersection_checker(self, m1, b1, m2, b2, initial_pose, target_pose):
        x1, y1 = initial_pose[0], initial_pose[1]
        x2, y2 = target_pose[0], target_pose[1]

        x_inter = (b2 - b1) / (m1 - m2) if m1 != m2 else None
        if x_inter is not None and (min(x1, x2) < x_inter < max(x1, x2)):
            return True
        return False

    def follow_path(self, path):
        for waypoint in path:
            self.move_to_waypoint(waypoint)
    
    def move_to_waypoint(self, waypoint):
        if self.turtle_pose:
            twist = Twist()
            angle_to_target = math.atan2(waypoint[1] - self.turtle_pose[1], waypoint[0] - self.turtle_pose[0])
            distance_to_target = math.sqrt((waypoint[0] - self.turtle_pose[0])**2 + (waypoint[1] - self.turtle_pose[1])**2)
            
            angle_diff = self.normalize_angle(angle_to_target - self.turtle_pose[2])
            if abs(angle_diff) > 0.1:
                twist.angular.z = 2.0 * (angle_diff / abs(angle_diff))  
                twist.linear.x = 0.0  
            else:
                twist.angular.z = 0.0
                twist.linear.x = 1.0 if distance_to_target > 0.1 else 0.0
            
            self.cmd_pub.publish(twist)
            self.rate.sleep()
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TurtleAStar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()