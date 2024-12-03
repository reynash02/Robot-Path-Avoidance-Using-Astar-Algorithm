import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


def cross_product(o, a, b):
    """Cross product of vectors OA and OB. Positive if counter-clockwise."""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def compute_convex_hull(points):
    """Compute the convex hull using the Graham Scan algorithm."""
    # Sort points lexicographically (by x, then by y)
    points = sorted(points)

    # Build the lower hull
    lower = []
    for p in points:
        while len(lower) >= 2 and cross_product(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Build the upper hull
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross_product(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenate lower and upper hulls, excluding the last point of each (repeated)
    return lower[:-1] + upper[:-1]


class ConvexHullPathPlanner(Node):
    def __init__(self):
        super().__init__('convex_hull_path_planner')

        # Publisher and subscriber
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)

        # Pose of the turtle
        self.turtle_pose = Pose()
        self.is_pose_received = False

        # Predefined points
        self.points = [(2.0, 2.0), (4.0, 2.0), (4.0, 8.0)]
        self.convex_hull_points = compute_convex_hull(self.points)

        self.get_logger().info(f"Convex Hull Points: {self.convex_hull_points}")

        # Timer to move through the points
        self.timer = self.create_timer(0.1, self.move_to_points)

    def update_pose(self, msg):
        """Update the current pose of the turtle."""
        self.turtle_pose = msg
        self.is_pose_received = True

    def euclidean_distance(self, goal_x, goal_y):
        """Calculate the Euclidean distance to the goal."""
        return math.sqrt((goal_x - self.turtle_pose.x) ** 2 + (goal_y - self.turtle_pose.y) ** 2)

    def linear_velocity(self, goal_x, goal_y, constant=0.5):
        """Calculate linear velocity towards the goal."""
        return constant * self.euclidean_distance(goal_x, goal_y)

    def steering_angle(self, goal_x, goal_y):
        """Calculate the angle to the goal."""
        return math.atan2(goal_y - self.turtle_pose.y, goal_x - self.turtle_pose.x)

    def angular_velocity(self, goal_theta, constant=2.0):
        """Calculate angular velocity towards the goal."""
        return constant * (goal_theta - self.turtle_pose.theta)

    def move_to_points(self):
        """Move the turtle to the points on the convex hull."""
        if not self.is_pose_received or not self.convex_hull_points:
            return

        # Get the next point
        goal_x, goal_y = self.convex_hull_points[0]

        # Calculate distance and angles
        distance = self.euclidean_distance(goal_x, goal_y)
        goal_theta = self.steering_angle(goal_x, goal_y)

        # Create velocity message
        msg = Twist()
        if distance >= 0.05:  # Stop condition
            msg.linear.x = self.linear_velocity(goal_x, goal_y, constant=0.2)
            msg.angular.z = self.angular_velocity(goal_theta, constant=1.0)
        else:
            self.get_logger().info(f'Reached point: ({goal_x}, {goal_y})')
            self.convex_hull_points.pop(0)  # Remove visited point

        # Publish velocity
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConvexHullPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
