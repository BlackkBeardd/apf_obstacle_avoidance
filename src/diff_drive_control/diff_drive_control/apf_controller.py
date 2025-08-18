import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt


class APF_controller(Node):
    def __init__(self):
        super().__init__("artificial_potential_field_controller")

        # Create publisher and subscriber
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.robot_pose_subscriber = self.create_subscription(
            Odometry, "/model/diff_drive/odometry", self.robot_pose_callback, 100
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan, "lidar", self.lidar_callback, 10
        )

        self.goal = None
        self.lidar_data = None
        self.dist = []
        self.dist_t = []
        # Declare and set parameters
        self.declare_parameter("max_linear_vel", 0.2)
        self.declare_parameter("goal", [0.0, 0.0])
        self.declare_parameter("kp", 0.1)
        self.declare_parameter("eta", 0.1)
        self.declare_parameter("repulsion_radius", 0.5)
        self.kp = self.get_parameter("kp").value
        self.eta = self.get_parameter("eta").value
        self.repulsion_radius = self.get_parameter("repulsion_radius").value
        self.max_linear_vel = self.get_parameter("max_linear_vel").value
        self.goal = np.array(self.get_parameter("goal").value)
        self.get_logger().info(f"goal: {self.goal}")
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Flag to check if robot pose is ready
        self.robot_pose_ready = False
        self.robot_orientation = 0.0
        self.robot_position = np.array([0.0, 0.0])  # Assume starting at
        self.ctrl_loop_timer = self.create_timer(0.01, self.control_loop)

    def robot_pose_callback(self, msg):
        """
        Updates the robot's position and orientation based on the given message.

        Parameters:
            msg (geometry_msgs.msg.PoseStamped): The message containing the robot's pose.

        Returns:
            None
        """
        self.robot_position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y]
        )

        # The pose contains the orientation in form of quaternion.
        # Convert quaternion to euler angles
        _, _, self.robot_orientation = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )

        self.robot_pose_ready = True

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def attractive_potential(self):
        if self.goal is None:
            return np.array([0.0, 0.0])

        # The attractive potential is the proportional distance towards the goal
        return self.kp * (self.goal - self.robot_position)

    def repulsive_potential(self):
        """
        Calculates the repulsive potential for the robot to avoid obstacles.

        Returns:
            np.array: The repulsive force to avoid obstacles.
        """
        if self.lidar_data is None:
            return np.array([0.0, 0.0])
        force = np.array([0.0, 0.0])
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment
        for i, distance in enumerate(self.lidar_data.ranges):
            if distance < self.repulsion_radius:
                angle = angle_min + i * angle_increment
                obstacle_pos = np.array(
                    [distance * np.cos(angle), distance * np.sin(angle)]
                )
                force += (
                    self.eta
                    * (1.0 / distance - 1.0 / self.repulsion_radius)
                    * (1.0 / (distance**2))
                    * (self.robot_position - obstacle_pos)
                    / np.linalg.norm(self.robot_position - obstacle_pos)
                )
        return force

    def get_dist_from_obstacles(self):
        if self.lidar_data is None:
            return 0
        return min(self.lidar_data.ranges)

    def control_loop(self):

        if self.goal is None:
            return

        # Calculate the resultant force
        attractive = self.attractive_potential()
        repulsive = self.repulsive_potential()
        resultant_force = attractive + repulsive

        self.get_logger().info(f"attractive force: {attractive}")
        self.get_logger().info(f"repulsive force: {repulsive}")
        self.get_logger().info(f"resultant force: {resultant_force}")

        # Calculate the velocity command
        angular_direction = math.atan2(resultant_force[1], resultant_force[0])
        angular_error = angular_direction - self.robot_orientation
        linear_error = math.sqrt(
            resultant_force[0] * resultant_force[0]
            + resultant_force[1] * resultant_force[1]
        )
        target_dist = math.sqrt(
            (self.goal[0] - self.robot_position[0]) ** 2
            + (self.goal[1] - self.robot_position[1]) ** 2
        )

        dist = self.get_dist_from_obstacles()
        if dist == 0:
            dist = 8.0  # Max lidar allowed values
        self.dist.append(dist)

        self.get_logger().info(
            f"angular_error: {angular_error}, linear_error: {linear_error}, target_dist: {target_dist}"
        )

        velocity = Twist()
        if target_dist > 0.05:

            # Move first the orientation if the linear error is too large
            if angular_error < 0.2:
                velocity.linear.x = linear_error * self.max_linear_vel
            else:
                velocity.linear.x = 0.0
            velocity.angular.z = angular_error
        else:
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0

            plt.plot(self.dist, color="red", linewidth=2.0)

            # Adding titles and labels
            plt.title("Distance from obstacles")
            plt.xlabel("Index")
            plt.ylabel("Distance")

            # Display the plot
            plt.show()

        self.publisher_.publish(velocity)

    def parameter_callback(self, params):
        for param in params:
            if param.name == "kp":
                self.kp = param.value
            elif param.name == "eta":
                self.eta = param.value
            elif param.name == "repulsion_radius":
                self.repulsion_radius = param.value
            elif param.name == "max_linear_vel":
                self.max_linear_vel = param.value
            elif param.name == "goal":
                self.goal = np.array(param.value)

        self.get_logger().info(
            f"kp: {self.kp}, eta: {self.eta}, repulsion_radius: {self.repulsion_radius}, max_linear_vel: {self.max_linear_vel}, goal: {self.goal}"
        )

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    controller = APF_controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
