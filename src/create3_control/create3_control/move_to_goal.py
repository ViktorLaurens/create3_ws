import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import random

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal')

        # Load the target position from YAML
        config_file = get_package_share_directory('create3_control') + '/target_position.yaml'
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
            self.target_position = config['target_position']

        # First go to a different target
        self.first_goal_reached = False
        self.target_position2 = self.generate_random_target()

        # Print the target position for now
        self.get_logger().info(f"Target position: {self.target_position}")
        
        # Subscribe to the /odom topic to get the current position
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher for the /cmd_vel topic to control the robot's movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Rotation tolerances in radians
        self.rotation_tolerance = 0.1  # Rotation tolerance in radians
        self.distance_tolerance = 0.01 # Distance tolerance in meter

    def generate_random_target(self):
        # Randomly choose an angle between 0 and 2π
        random_angle = random.uniform(0, 2 * math.pi)

        # Randomly choose a distance from 0 to 2 meters
        random_radius = random.uniform(0, 2)  # Random distance within 2 meters

        # Convert polar coordinates to Cartesian coordinates
        x = random_radius * math.cos(random_angle)
        y = random_radius * math.sin(random_angle)

        target_position = {'x': x, 'y': y}
        self.get_logger().info(f"Random target position: {target_position}")
        return target_position

    def odom_callback(self, msg):
        if not self.first_goal_reached:
            # Extract position data from the odometry message
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            # Log the current position
            self.get_logger().info(f"Current position: x={position.x}, y={position.y}")

            # Calculate the yaw angle from the quaternion
            _, _, current_yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            self.get_logger().info(f"Current yaw angle: {current_yaw} rad")

            # Calculate the desired yaw angle toward the target position
            desired_yaw = self.calculate_desired_yaw(position.x, position.y, self.target_position2)
            self.get_logger().info(f"Desired yaw angle: {desired_yaw} rad")

            # Calculate the difference between the desired and current yaw
            yaw_error = desired_yaw - current_yaw

            # Normalize the yaw error to the range [-pi, pi]
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

            if abs(yaw_error) < self.rotation_tolerance:
                # self.get_logger().info("Reached desired orientation. Stopping the robot and shutting down the node.")
                
                # Calculate the distance to the target position
                target_x = self.target_position2['x']
                target_y = self.target_position2['y']
        
                dx = target_x - position.x
                dy = target_y - position.y
                distance = (dx ** 2 + dy ** 2) ** 0.5

                if distance < self.distance_tolerance: 
                    self.get_logger().info("Reached target position. Stopping the robot.")

                    # Stop the robot by publishing a Twist message with zero velocities
                    stop_cmd = Twist()
                    self.cmd_vel_publisher.publish(stop_cmd)
                    
                    self.first_goal_reached = True
                else: 
                    self.move_towards_target()
            else:
                # If not at desired orientation, rotate the robot towards the target
                self.rotate_to_target(yaw_error)
        else:  
            # Extract position data from the odometry message
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            # Log the current position
            self.get_logger().info(f"Current position: x={position.x}, y={position.y}")

            # Calculate the yaw angle from the quaternion
            _, _, current_yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            self.get_logger().info(f"Current yaw angle: {current_yaw} rad")

            # Calculate the desired yaw angle toward the target position
            desired_yaw = self.calculate_desired_yaw(position.x, position.y, self.target_position)
            self.get_logger().info(f"Desired yaw angle: {desired_yaw} rad")

            # Calculate the difference between the desired and current yaw
            yaw_error = desired_yaw - current_yaw

            # Normalize the yaw error to the range [-pi, pi]
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

            if abs(yaw_error) < self.rotation_tolerance:
                # self.get_logger().info("Reached desired orientation. Stopping the robot and shutting down the node.")
                
                # Calculate the distance to the target position
                target_x = self.target_position['x']
                target_y = self.target_position['y']
        
                dx = target_x - position.x
                dy = target_y - position.y
                distance = (dx ** 2 + dy ** 2) ** 0.5

                if distance < self.distance_tolerance: 
                    self.get_logger().info("Reached target position. Stopping the robot.")

                    # Stop the robot by publishing a Twist message with zero velocities
                    stop_cmd = Twist()
                    self.cmd_vel_publisher.publish(stop_cmd)
                    
                    # Shutdown the node
                    rclpy.shutdown()
                else: 
                    self.move_towards_target()
            else:
                # If not at desired orientation, rotate the robot towards the target
                self.rotate_to_target(yaw_error)
 
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def calculate_desired_yaw(self, current_x, current_y, target_position):
        """
        Calculate the desired yaw angle to face the target position.
        """
        target_x = target_position['x']
        target_y = target_position['y']
        return math.atan2(target_y - current_y, target_x - current_x)

    def rotate_to_target(self, yaw_error):
        """
        Rotate the robot towards the target position by publishing Twist messages.
        """
        twist = Twist()
        twist.angular.z = 0.5 * yaw_error  # Adjust the gain (0.5) as needed
        self.cmd_vel_publisher.publish(twist) # Publish the Twist message

    def move_towards_target(self): 
        # Move towards the target by publishing a forward velocity
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # Set the speed (adjust as needed)
        self.cmd_vel_publisher.publish(move_cmd)    # Publish the move command     


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
