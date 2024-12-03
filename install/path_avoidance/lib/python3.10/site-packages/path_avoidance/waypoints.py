import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import time

class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')

        self.max_value = 4.0
        self.avoid = False
        self.theta = 0.0
        self.waypoints = [(1.5, 1.5), (8.5, 8.5), (5.0, 5.0)]  # Example waypoints
        self.current_waypoint = 0
        self.previous_positions = []  # List to store previous positions for line avoidance
        
        self.drive_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.pose = Pose()  # Initialize the pose attribute
        self.timer = self.create_timer(0.1, self.timer_callback)  # Call every 0.1 seconds

    def pose_callback(self, pose: Pose):
        # Update the current pose of the turtle
        self.pose = pose
        if self.is_near_border(pose.x, pose.y):
            self.avoid = True
            self.theta = pose.theta
            self.get_logger().warn(f"Near border! Avoiding collision at position: ({pose.x}, {pose.y})")

    def is_near_border(self, x, y):
        return x <= 1.0 or x >= 10.0 or y <= 1.0 or y >= 10.0

    def calculate_next_move(self):
        # Calculate random linear and angular velocities
        linear_x = random.uniform(0, self.max_value)
        angular_z = random.uniform(-self.max_value, self.max_value)
        return linear_x, angular_z

    def avoid_collision(self):
        # Stop the turtle
        drive = Twist()
        self.drive_pub.publish(drive)
        
        # Move backwards for a short time
        drive.linear.x = -0.5
        self.drive_pub.publish(drive)
        time.sleep(2)
        
        # Turn left or right randomly
        drive.linear.x = 0.0
        direction = random.choice([-1, 1])
        drive.angular.z = direction * 3.6
        self.drive_pub.publish(drive)
        time.sleep(2)

        # Re-start movement after collision avoidance
        self.avoid = False

    def move_turtle(self):
        drive = Twist()

        # Move to current waypoint
        if self.current_waypoint < len(self.waypoints):
            x_target, y_target = self.waypoints[self.current_waypoint]
            self.get_logger().info(f"Moving to waypoint: ({x_target}, {y_target})")

            # Move towards the target point
            drive.linear.x, drive.angular.z = self.calculate_next_move()
            self.drive_pub.publish(drive)

            # Check if turtle is close to the target point
            if self.is_at_target(x_target, y_target):
                self.get_logger().info(f"Reached waypoint: ({x_target}, {y_target})")
                self.previous_positions.append((x_target, y_target))  # Store passed position
                self.current_waypoint += 1  # Move to next waypoint

    def is_at_target(self, x_target, y_target):
        # Define the threshold for reaching the target (distance tolerance)
        tolerance = 0.2
        if abs(x_target - self.pose.x) < tolerance and abs(y_target - self.pose.y) < tolerance:
            return True
        return False

    def timer_callback(self):
        if self.avoid:
            self.avoid_collision()
        else:
            self.move_turtle()

def main(args=None):
    rclpy.init(args=args)

    turtle_mover = TurtleMover()

    try:
        rclpy.spin(turtle_mover)
    except KeyboardInterrupt:
        pass

    turtle_mover.destroy_node()
    rclpy.shutdown()
