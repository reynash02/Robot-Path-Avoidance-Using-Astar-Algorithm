import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class SimpleGreedyPathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Initialize publisher and subscriber
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        
        # Initialize turtle pose
        self.turtle_pose = Pose()
        self.is_pose_received = False
        
        # Predefined points
        self.points = [(2.0, 2.0), (8.0, 2.0), (5.0, 8.0)]  # Example points
        self.visited_points = []
        
        # Timer to control movement
        self.timer = self.create_timer(0.1, self.move_to_points)

    def update_pose(self, msg):
        """Update the current pose of the turtle."""
        self.turtle_pose = msg
        self.is_pose_received = True

    def euclidean_distance(self, goal_x, goal_y):
        """Calculate Euclidean distance to the goal."""
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
        """Move the turtle to the predefined points."""
        if not self.is_pose_received or not self.points:
            return

        # Get the next point
        goal_x, goal_y = self.points[0]
        
        # Calculate distance and angles
        distance = self.euclidean_distance(goal_x, goal_y)
        goal_theta = self.steering_angle(goal_x, goal_y)

        # Create velocity message
        msg = Twist()
        if distance >= 0.05:  # Reduce tolerance for better precision
            msg.linear.x = self.linear_velocity(goal_x, goal_y, constant=0.3)  # Slightly increased speed
            msg.angular.z = self.angular_velocity(goal_theta, constant=1.5)
        else:
            self.get_logger().info(f'Reached point: ({goal_x}, {goal_y})')
            self.visited_points.append((goal_x, goal_y))
            self.points.pop(0)  # Remove the visited point
        
        # Publish velocity
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGreedyPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
