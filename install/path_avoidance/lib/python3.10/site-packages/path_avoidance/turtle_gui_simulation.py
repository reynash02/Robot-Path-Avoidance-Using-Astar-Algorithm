import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

class PathLogger(Node):
    def __init__(self):
        super().__init__("path_logger")
        self.visited_points = [np.array([5.5, 5.5])]  # Initial point (center of turtlesim screen)
        self.current_position = np.array([5.5, 5.5])
        self.current_orientation = 0.0  # Current orientation in radians
        self.path_resolution = 0.05
        self.target_position = None
        self.obstacle_margin = 0.2
        self.reached_tolerance = 0.2

        # ROS2 publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # Timer for movement updates
        self.timer = self.create_timer(0.1, self.update)

    def pose_callback(self, msg):
        """Callback to update the turtle's current position and orientation."""
        self.current_position = np.array([msg.x, msg.y])
        self.current_orientation = msg.theta

    def set_target(self, x, y):
        """Set the target position for the turtle."""
        self.target_position = np.array([x, y])
        self.get_logger().info(f"Target position set to: {self.target_position}")

    def interpolate_path(self, start, end):
        """Interpolate between two points with fixed resolution."""
        t = np.linspace(0, 1, int(np.linalg.norm(end - start) / self.path_resolution))
        return np.array([start + t_val * (end - start) for t_val in t])

    def check_intersection(self, new_path):
        """Check if the new path intersects with any visited paths."""
        for i in range(len(self.visited_points) - 1):
            segment_start = self.visited_points[i]
            segment_end = self.visited_points[i + 1]
            existing_path = self.interpolate_path(segment_start, segment_end)
            kdtree = KDTree(existing_path)
            new_path = np.array(new_path)
            if new_path.ndim == 1:
                new_path = new_path.reshape(-1, 2)  # Ensure it's a 2D array if it's 1D

            distances, _ = kdtree.query(new_path, k=1)
            if np.any(distances < self.obstacle_margin):
                return True
        return False

    def find_alternate_path(self, start, end, depth=0, max_depth=5):
        """Find an alternate path using recursive multi-step detours."""
        if depth > max_depth:
            self.get_logger().error("Max recursive depth reached. No alternate path found.")
            return None
    
        # Try grid-based detour search
        direction = end - start
        step_size = self.obstacle_margin * 0.5
        max_attempts = 20  # Increased attempts
        angle_steps = 16
    
        for attempt in range(1, max_attempts + 1):
            radius = attempt * step_size
            for angle in np.linspace(0, 2 * np.pi, angle_steps, endpoint=False):
                offset = radius * np.array([np.cos(angle), np.sin(angle)])
                waypoint = start + direction * 0.5 + offset
    
                # Check if the path to the waypoint is clear
                to_waypoint_path = self.interpolate_path(start, waypoint)
                from_waypoint_path = self.interpolate_path(waypoint, end)
    
                if not self.check_intersection(to_waypoint_path) and not self.check_intersection(from_waypoint_path):
                    self.get_logger().info(f"Found alternate waypoint at depth {depth}: {waypoint}")
                    return [waypoint]
    
        # If no single detour works, divide and conquer
        mid_point = (start + end) / 2
        self.get_logger().info(f"Attempting recursive search at depth {depth + 1}")
        first_leg = self.find_alternate_path(start, mid_point, depth + 1, max_depth)
        second_leg = self.find_alternate_path(mid_point, end, depth + 1, max_depth)
    
        if first_leg and second_leg:
            return first_leg + second_leg
    
        return None


    def plan_path(self, target):
        """Plan a path to the target position."""
        self.get_logger().info(f"Target position set to: {target}")
        path = self.interpolate_path(self.current_position, target)

        if self.check_intersection(path):
            self.get_logger().info("Path intersects with previous paths. Re-routing...")
            detour = self.find_alternate_path(self.current_position, target)
            if detour:
                detour_path = [self.current_position] + detour + [target]
                self.visited_points.extend(detour)  # Add detour points to visited paths
                return detour_path
            else:
                self.get_logger().error("No viable path found. Stopping...")
                return None
        else:
            self.visited_points.append(target)
            return [target]

    def move_turtle(self):
        """Move the turtle towards the target position."""
        if self.target_position is None:
            return

        direction = self.target_position - self.current_position
        distance = np.linalg.norm(direction)

        if distance <= self.reached_tolerance:
            self.get_logger().info("Reached target position.")
            self.visited_points.append(self.current_position.copy())
            self.target_position = None
            return

        # Compute angle to target
        target_angle = np.arctan2(direction[1], direction[0])
        angle_diff = target_angle - self.current_orientation

        # Normalize angle difference to [-pi, pi]
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

        twist = Twist()
        if abs(angle_diff) > 0.05:  # Rotate to align
            twist.angular.z = 1.5 * np.sign(angle_diff) * min(abs(angle_diff), 1.0)
        else:  # Move forward once aligned
            twist.linear.x = min(distance, 1.0)
        self.cmd_vel_publisher.publish(twist)

    def update(self):
        self.move_turtle()

        # Plotting the paths
        plt.clf()
        for i in range(len(self.visited_points) - 1):
            segment_start = self.visited_points[i]
            segment_end = self.visited_points[i + 1]
            plt.plot([segment_start[0], segment_end[0]], 
                     [segment_start[1], segment_end[1]], "b-")
        if self.target_position is not None:
            plt.scatter(self.target_position[0], self.target_position[1], c="g", label="Target Position")
        plt.scatter(self.current_position[0], self.current_position[1], c="r", label="Current Position")
        plt.title("Turtlesim Path Logger with Avoidance")
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.legend()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PathLogger()
    plt.ion()
    plt.figure()

    try:
        # Example target positions
        user_targets = [[2.0, 2.0], [8.0, 8.0], [3.0, 7.0]]
        for target in user_targets:
            planned_path = node.plan_path(np.array(target))
            if planned_path:
                node.set_target(target[0], target[1])
                while node.target_position is not None:
                    rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        plt.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
