import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import math
import time


class PathAvoidance(Node):
    def __init__(self):
        super().__init__('path_avoidance')
        
        # Subscriptions and Publications
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Cost Map: Representing the grid as a 2D numpy array
        self.grid_size = 11  # Turtlesim runs in a 0-11 coordinate space
        self.cost_map = np.zeros((self.grid_size, self.grid_size))
        
        # Current pose
        self.pose = None
        self.prev_pose = None
        
        # Movement parameters
        self.points = [(1, 1), (1, 2), (6, 2)]  # Target points
        self.target_index = 0
        self.goal_tolerance = 0.1  # Tighter tolerance to avoid moving too far past the point
        self.timer = self.create_timer(0.1, self.move_to_goal)  # Smaller period for faster checks

    def pose_callback(self, msg):
        self.pose = msg

    def move_to_goal(self):
        if not self.pose or self.target_index >= len(self.points):
            return

        # Get current target
        target_x, target_y = self.points[self.target_index]

        # Check if the turtle is near the goal
        if self.distance_to_goal(target_x, target_y) < self.goal_tolerance:
            self.get_logger().info(f"Reached point: {target_x}, {target_y}")
            self.mark_path_on_cost_map()
            self.target_index += 1
            if self.target_index >= len(self.points):
                self.stop_turtle()
                self.get_logger().info("All points reached!")
            return

        # Calculate heading and check for collisions
        angle_to_target = math.atan2(target_y - self.pose.y, target_x - self.pose.x)
        next_x = int(self.pose.x + math.cos(angle_to_target))
        next_y = int(self.pose.y + math.sin(angle_to_target))

        if self.is_collision(next_x, next_y):
            # Rotate to find a new direction
            self.get_logger().info("Collision detected! Adjusting path...")
            self.avoid_collision()
        else:
            # Move towards the target with a very slow speed, updating cost map after each small move
            twist = Twist()
            twist.linear.x = 0.1  # Very slow forward speed for better control
            twist.angular.z = 0.3 * (angle_to_target - self.pose.theta)  # Adjust heading slightly
            self.vel_pub.publish(twist)

            # Wait for a brief period to allow the move to happen
            time.sleep(0.2)  # Sleep for 0.2 seconds to allow movement and calculation

            # Mark the new position on the cost map after each small movement
            self.mark_path_on_cost_map()

    def avoid_collision(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Rotate slowly
        self.vel_pub.publish(twist)

    def distance_to_goal(self, x, y):
        return math.sqrt((x - self.pose.x) ** 2 + (y - self.pose.y) ** 2)

    def is_collision(self, x, y):
        # Check multiple steps ahead to ensure no collision with the marked path
        x1, y1 = int(self.pose.x), int(self.pose.y)
        x2, y2 = x, y
        for x, y in self.bresenham(x1, y1, x2, y2):
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                if self.cost_map[y, x] == 1:  # Use (y, x) for numpy indexing
                    return True
        return False

    def mark_path_on_cost_map(self):
        # Mark the path between the previous position and the current position
        if hasattr(self, 'prev_pose'):
            x1, y1 = int(self.prev_pose.x), int(self.prev_pose.y)
            x2, y2 = int(self.pose.x), int(self.pose.y)
            
            # Bresenham's line algorithm to mark cells
            for x, y in self.bresenham(x1, y1, x2, y2):
                if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                    self.cost_map[y, x] = 1  # Use (y, x) for numpy indexing
        
        # Update previous pose
        self.prev_pose = self.pose

    def bresenham(self, x1, y1, x2, y2):
        """Generate points on a line using Bresenham's algorithm."""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            points.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy

        return points

    def stop_turtle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PathAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
