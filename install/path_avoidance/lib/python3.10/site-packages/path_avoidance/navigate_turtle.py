import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class NoIntersectionPathPlanner(Node):
    def __init__(self):
        super().__init__('no_intersection_path_planner')

        # Publisher and subscriber
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)

        # Pose of the turtle
        self.turtle_pose = Pose()
        self.is_pose_received = False

        # Predefined points
        self.points = [(2.0, 2.0), (8.0, 2.0), (5.0, 8.0), (3.0, 6.0)]
        self.visited_points = []
        self.lines = []  # List of lines as (x1, y1, x2, y2)

        # Timer to move through the points
        self.timer = self.create_timer(0.1, self.move_to_points)

    def update_pose(self, msg):
        """Update the current pose of the turtle."""
        self.turtle_pose = msg
        self.is_pose_received = True

    def euclidean_distance(self, goal_x, goal_y):
        """Calculate the Euclidean distance to the goal."""
        return math.sqrt((goal_x - self.turtle_pose.x) ** 2 + (goal_y - self.turtle_pose.y) ** 2)

    def linear_velocity(self, goal_x, goal_y, constant=0.2):
        """Calculate linear velocity towards the goal."""
        return constant * self.euclidean_distance(goal_x, goal_y)

    def steering_angle(self, goal_x, goal_y):
        """Calculate the angle to the goal."""
        return math.atan2(goal_y - self.turtle_pose.y, goal_x - self.turtle_pose.x)

    def angular_velocity(self, goal_theta, constant=2.0):
        """Calculate angular velocity towards the goal."""
        return constant * (goal_theta - self.turtle_pose.theta)

    def is_point_on_segment(self, point, seg_start, seg_end):
        """Check if a point is on the line segment."""
        x, y = point
        x1, y1 = seg_start
        x2, y2 = seg_end

        # Check if the point is within the bounds of the segment
        if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
            return True
        return False

    def do_segments_intersect(self, seg1, seg2):
        """Check if two line segments intersect."""
        x1, y1, x2, y2 = seg1
        x3, y3, x4, y4 = seg2

        # Denominator for the intersection point
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            return False  # Lines are parallel

        # Calculate the intersection point
        intersect_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
        intersect_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

        # Check if the intersection point is within both line segments
        if self.is_point_on_segment((intersect_x, intersect_y), (x1, y1), (x2, y2)) and self.is_point_on_segment((intersect_x, intersect_y), (x3, y3), (x4, y4)):
            return True
        return False

    def move_around_obstacle(self, current_point, target_point):
        """Move the turtle around an obstacle by avoiding intersection with previous lines."""
        # Try to move around the obstacle by offsetting the target point
        # (this is a basic approach to move around the obstacle)
        offset = 0.5
        new_target_x = target_point[0] + offset
        new_target_y = target_point[1] + offset
        return new_target_x, new_target_y

    def move_to_points(self):
        """Move the turtle to the predefined points."""
        if not self.is_pose_received or not self.points:
            return

        # Get the next point
        goal_x, goal_y = self.points[0]
        current_point = (self.turtle_pose.x, self.turtle_pose.y)
        next_point = (goal_x, goal_y)

        # Check if the move intersects with any previously drawn lines
        for line in self.lines:
            if self.do_segments_intersect((current_point[0], current_point[1], next_point[0], next_point[1]), line):
                self.get_logger().info(f"Intersection detected, avoiding obstacle!")
                # Move around the obstacle
                next_point = self.move_around_obstacle(current_point, next_point)
                break

        # Calculate distance and angles
        distance = self.euclidean_distance(goal_x, goal_y)
        goal_theta = self.steering_angle(goal_x, goal_y)

        # Create velocity message
        msg = Twist()
        if distance >= 0.05:  # Stop condition
            msg.linear.x = self.linear_velocity(goal_x, goal_y, constant=0.1)  # Reduced speed
            msg.angular.z = self.angular_velocity(goal_theta, constant=1.0)
        else:
            self.get_logger().info(f'Reached point: ({goal_x}, {goal_y})')
            self.visited_points.append((goal_x, goal_y))
            self.lines.append((current_point[0], current_point[1], next_point[0], next_point[1]))  # Add the line as a segment
            self.points.pop(0)  # Remove visited point

        # Publish velocity
        self.publisher.publish(msg)

    def visualize_lines(self):
        """Visualize the lines drawn by the turtle."""
        for line in self.lines:
            self.get_logger().info(f"Line segment: {line}")


def main(args=None):
    rclpy.init(args=args)
    node = NoIntersectionPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
