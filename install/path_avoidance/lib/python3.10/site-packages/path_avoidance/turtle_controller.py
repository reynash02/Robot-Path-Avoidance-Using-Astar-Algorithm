import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        self.target_points = [(2, 2), (2, 4), (7, 4)]  # Define target points
        self.current_point_index = 0
        self.path_segments = []  # Store path as a list of segments
        self.previous_pose = None

        self.timer = self.create_timer(0.1, self.timer_callback)  # Call timer callback at 10Hz

    def pose_callback(self, msg):
        self.current_pose = (msg.x, msg.y)

    def timer_callback(self):
        if self.current_point_index < len(self.target_points):
            target = self.target_points[self.current_point_index]
            
            # Check if the turtle is near the target
            if self.is_near(self.current_pose, target):
                self.current_point_index += 1
                if self.current_point_index < len(self.target_points):
                    self.previous_pose = self.current_pose  # Store the last position
                    self.path_segments.append(self.previous_pose)  # Store the previous point
                else:
                    self.get_logger().info('All points visited. Stopping turtle.')
                    self.stop_turtle()

            else:
                # Move towards the target point while avoiding crossing paths
                new_position = self.calculate_new_position(self.current_pose, target)
                if not self.would_cross_path(self.previous_pose, new_position):
                    self.publish_movement(new_position)

    def publish_movement(self, new_position):
        twist_msg = Twist()
        twist_msg.linear.x = new_position[0] - self.current_pose[0]
        twist_msg.linear.y = new_position[1] - self.current_pose[1]
        twist_msg.angular.z = 0.0  # Keep angle steady
        self.publisher.publish(twist_msg)

    def calculate_new_position(self, current_pose, target):
        # Simple proportional movement towards the target
        dx = target[0] - current_pose[0]
        dy = target[1] - current_pose[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance < 0.1:  # If very close, remain at current position
            return current_pose

        # Normalize direction and move towards the target a small step
        step_size = 0.1
        dx /= distance
        dy /= distance
        new_x = current_pose[0] + step_size * dx
        new_y = current_pose[1] + step_size * dy

        return (new_x, new_y)
        
    def would_cross_path(self, current_position, new_position):
        if current_position is None or new_position is None:
            return False

        for i in range(len(self.path_segments) - 1):
            p1 = self.path_segments[i]
            p2 = self.path_segments[i + 1]
            if self.lines_intersect(p1, p2, current_position, new_position):
                return True
        return False

    def lines_intersect(self, p1, p2, p3, p4):
        """ Check if line segments p1p2 and p3p4 intersect """
        def orientation(p, q, r):
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0  # collinear
            return 1 if val > 0 else 2  # clock or counterclockwise

        o1 = orientation(p1, p2, p3)
        o2 = orientation(p1, p2, p4)
        o3 = orientation(p3, p4, p1)
        o4 = orientation(p3, p4, p2)

        # General case
        return o1 != o2 and o3 != o4

    def stop_turtle(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

    def is_near(self, current_pose, target, tolerance=0.1):
        distance = math.sqrt((target[0] - current_pose[0])**2 + (target[1] - current_pose[1])**2)
        return distance <= tolerance


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()