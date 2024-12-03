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
        self.points = [(2.0, 2.0), (2.0, 4.0), (6.0, 4.0)]
        self.visited_points = []
        self.lines = []  # List of lines in equation form [(A, B, C)]

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

    def line_equation(self, p1, p2):
        """Calculate the equation of the line in the form Ax + By + C = 0"""
        x1, y1 = p1
        x2, y2 = p2
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2
        return A, B, C

    def is_point_on_line(self, point, line_eq):
        """Check if the point lies on the line defined by the equation Ax + By + C = 0."""
        A, B, C = line_eq
        x, y = point
        return A * x + B * y + C == 0

    def is_valid_move(self, current_point, next_point):
        """Check if the move intersects any of the previously drawn lines."""
        for line in self.lines:
            if self.is_point_on_line(next_point, line):
                return False
        return True

    def find_buffer_point(self, intersection_point, target_point):
        """Find a point near the intersection to go around the obstacle."""
        # Move perpendicular to the line at the intersection point
        angle = math.atan2(target_point[1] - intersection_point[1], target_point[0] - intersection_point[0])
        buffer_x = intersection_point[0] + math.cos(angle + math.pi / 2) * 0.5
        buffer_y = intersection_point[1] + math.sin(angle + math.pi / 2) * 0.5
        return buffer_x, buffer_y

    def move_to_points(self):
        """Move the turtle to the predefined points."""
        if not self.is_pose_received or not self.points:
            return

        # Get the next point
        goal_x, goal_y = self.points[0]
        current_point = (self.turtle_pose.x, self.turtle_pose.y)
        next_point = (goal_x, goal_y)

        # Check if the move is valid
        if not self.is_valid_move(current_point, next_point):
            self.get_logger().info(f"Path blocked, finding buffer point...")
            # Find a buffer point and move there first
            buffer_point = self.find_buffer_point(current_point, next_point)
            next_point = buffer_point

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
            self.lines.append(self.line_equation(current_point, next_point))  # Add the line equation
            self.points.pop(0)  # Remove visited point

        # Publish velocity
        self.publisher.publish(msg)

    def visualize_lines(self):
        """Visualize the lines drawn by the turtle."""
        for line in self.lines:
            self.get_logger().info(f"Line segment equation: {line}")


def main(args=None):
    rclpy.init(args=args)
    node = NoIntersectionPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
