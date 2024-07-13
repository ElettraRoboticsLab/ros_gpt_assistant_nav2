import math
from copy import deepcopy
from time import sleep
from typing import Union

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped


class ContextNavigator(BasicNavigator):
    """
    A navigator that extends the BasicNavigator class to provide additional functionalities
    Base class: https://github.com/open-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/robot_navigator.py
    """

    def __init__(
            self,
            node_name: str = "contex_navigator",
    ):
        super().__init__(node_name=node_name)

        self.current_odom_topic: str = self.declare_parameter("current_odom_topic", "/diff_cont/odom").value
        self.behavior_tree: str = self.declare_parameter("behavior_tree", "").value
        self.wait_for_nav2: bool = self.declare_parameter("wait_for_nav2", True).value

        self.current_odom: Odometry | None = None
        self.current_odom_received: bool = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_odom_sub = self.create_subscription(
            Odometry,
            self.current_odom_topic,
            self._current_odom_callback,
            10,
        )

        if self.wait_for_nav2:
            self.info("Waiting for Nav2 to become active")
            self.waitUntilNav2Active()
            self.info("Nav2 is active")

        if not self.current_odom_received:
            self.error("Current odom not received yet")
            raise Exception("Current odom not received yet")

    def spin_node(self):
        while rclpy.ok():
            self.spin_once()

    def spin_once(self):
        rclpy.spin_once(self)

    def destroy_node(self) -> None:
        self.info("Cancel task and destroying node")
        self.cancelTask()
        super().destroy_node()

    def get_current_pose_tuple(self) -> Union[tuple[float, float, float], tuple[None, None, None]]:
        current_pose = self.get_transform_pose_frame()
        if current_pose is None:
            return None, None, None
        return (
            current_pose.position.x,
            current_pose.position.y,
            self._quaternion_to_degree(current_pose.orientation)
        )

    def spin_to_relative(self, *, orientation: float, time_allowance: int = 5, wait: bool = True) -> None:
        radiant_angle = math.radians(orientation)
        self.info(f"spin_to_relative: orientation={orientation}")
        self.spin(spin_dist=radiant_angle, time_allowance=time_allowance)
        if wait:
            self.wait_until_task_complete()

    def go_to_absolute(self, *, x: float = None, y: float = None, orientation: float = None, wait: bool = True) -> None:
        goal = self._to_pose_stamped(x=x, y=y, orientation=orientation)
        self.info(f"go_to_absolute: x={goal.pose.position.x}, "
                  f"y={goal.pose.position.y}, orientation="
                  f"{self._quaternion_to_degree(goal.pose.orientation)}")
        self.goToPose(goal, behavior_tree=self.behavior_tree)
        if wait:
            self.wait_until_task_complete()

    def go_to_relative(self, *, x: float = None, y: float = None, orientation: float = None, wait: bool = True) -> None:
        goal = self._relative_to_pose_stamped(x=x, y=y, orientation=orientation)
        self.info(f"go_to_relative: x={x}, y={y}, orientation={orientation}")
        self.goToPose(goal, behavior_tree=self.behavior_tree)
        if wait:
            self.wait_until_task_complete()

    def go_through_absolute_poses(self, list_of_poses: list[dict], wait: bool = True) -> None:
        goal_poses = []
        for pose in list_of_poses:
            goal_poses.append(self._to_pose_stamped(**pose))
        self.info(f"go_through_absolute_poses: poses={list_of_poses}")
        # self.goThroughPoses(goal_poses)
        self.followWaypoints(goal_poses)
        if wait:
            self.wait_until_task_complete()

    def go_through_relative_poses(self, list_of_poses: list[dict], wait: bool = True) -> None:
        goal_poses = []
        for pose in list_of_poses:
            goal_poses.append(self._relative_to_pose_stamped(**pose))
        self.info(f"go_through_relative_poses: poses={list_of_poses}")
        # self.goThroughPoses(goal_poses)
        self.followWaypoints(goal_poses)
        if wait:
            self.wait_until_task_complete()

    def move_relative(self, *, x: float = None, y: float = None, orientation: float = None, wait: bool = True) -> None:
        current_x, current_y, current_orientation = self.get_current_pose_tuple()
        self.info(f"move_relative: current position x={current_x}, y={current_y}, orientation={current_orientation}")

        if x is None:
            x = 0.0
        if y is None:
            y = 0.0
        if orientation is None:
            orientation = 0.0

        updated_orientation = orientation + current_orientation
        calc_orientation = deepcopy(updated_orientation) + 90.0

        # Calculate the distance to move forward in the current orientation
        distance = math.sqrt(x ** 2 + y ** 2)

        # Determine the angle of movement in radians relative to the original position
        move_angle = math.atan2(y, x)

        # Adjust the move angle by the current orientation
        adjusted_angle = math.radians(calc_orientation) + move_angle

        # Calculate the new position based on distance and orientation
        updated_x = current_x + distance * math.cos(adjusted_angle)
        updated_y = current_y + distance * math.sin(adjusted_angle)

        self.info(f"move_relative: calculated new position "
                  f"x={updated_x}, y={updated_y}, orientation={updated_orientation}")

        self.go_to_absolute(
            x=updated_x,
            y=updated_y,
            orientation=updated_orientation,
            wait=wait
        )

    def wait_until_task_complete(self) -> None:
        self.info("Waiting for task to complete")
        while not self.isTaskComplete():
            sleep(0.333)
        self.info("Task complete")

    def cancel_task(self) -> None:
        self.cancelTask()

    def _to_pose_stamped(self, *, x: float = None, y: float = None, orientation: float = None) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = "map"  # TODO: ? self.current_odom.header.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()

        # Copy current pose to goal odom
        goal.pose = self.current_odom.pose.pose

        # Update goal pose with new position
        if x is not None:
            goal.pose.position.x = x
        if y is not None:
            goal.pose.position.y = y
        if orientation is not None:
            goal.pose.orientation = self._degree_to_quaternion(orientation)

        return deepcopy(goal)

    def _relative_to_pose_stamped(self, *, x: float = None, y: float = None, orientation: float = None) -> PoseStamped:
        current_x, current_y, current_orientation = self.get_current_pose_tuple()

        return self._to_pose_stamped(
            x=current_x + (x if x is not None else 0.0),
            y=current_y + (y if y is not None else 0.0),
            orientation=current_orientation + (orientation if orientation is not None else 0.0)
        )

    def _current_odom_callback(self, msg: Odometry) -> None:
        self.current_odom = msg
        if not self.current_odom_received:
            self.info("Received first odom")
            self.current_odom_received = True

    def get_transform_pose_frame(self, target_frame="map", robot_frame="base_link") -> Union[Pose, None]:
        if self.current_odom is None:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                robot_frame,
                rclpy.time.Time()
            )
            self.info(f"Transform: {transform}")
            self.info(f"TO pose stamped {self._to_pose_stamped()}")

            transformed_pose = tf2_geometry_msgs.do_transform_pose(self._to_pose_stamped().pose, transform)
            self.info(f"Transformed pose: {transformed_pose}")
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.error(f"TF2 Exception: {e}")
            return None

    @staticmethod
    def _quaternion_to_degree(quaternion: Quaternion) -> float:
        # Calculate the rotation about the Z-axis
        theta_rad = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        )

        # Convert angle from radians to degrees
        theta_deg = math.degrees(theta_rad)

        return round(theta_deg, 4)

    @staticmethod
    def _degree_to_quaternion(theta_deg: float) -> Quaternion:
        # Convert angle from degrees to radians
        theta_rad = math.radians(theta_deg)

        # Calculate half angle
        half_theta = theta_rad / 2.0

        # Calculate sine and cosine of half angle
        sin_half_theta = math.sin(half_theta)
        cos_half_theta = math.cos(half_theta)

        # Construct quaternion representing rotation about Z-axis
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = round(sin_half_theta, 4)
        quaternion.w = round(cos_half_theta, 4)

        return deepcopy(quaternion)
