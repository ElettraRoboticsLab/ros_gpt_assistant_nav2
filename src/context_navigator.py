import math
from copy import deepcopy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry


class ContexNavigator(BasicNavigator):
    """
    A navigator that extends the BasicNavigator class to provide additional functionalities
    Base class: https://github.com/open-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/robot_navigator.py
    """

    def __init__(
            self,
            node_name: str = "contex_navigator",
            current_pose_topic: str = "/diff_cont/odom",
            behavior_tree: str = "",
            wait_for_nav2: bool = True,
    ):
        super().__init__(node_name=node_name)

        self.behavior_tree: str = behavior_tree
        self.current_pose: Odometry | None = None
        self.current_pose_received: bool = False
        self.current_pose_topic: str = current_pose_topic

        self.current_pose_sub = self.create_subscription(
            Odometry,
            self.current_pose_topic,
            self._current_pose_callback,
            10,
        )

        if wait_for_nav2:
            self.info("Waiting for Nav2 to become active")
            self.waitUntilNav2Active()
            self.info("Nav2 is active")

        if not self.current_pose_received:
            self.error("Current pose not received yet")
            raise Exception("Current pose not received yet")

    def destroy_node(self) -> None:
        self.info("Destroying node")
        self.cancelTask()
        super().destroy_node()

    def spin_to_relative(self, *, orientation: float, time_allowance: int = 5, wait: bool = True) -> None:
        radiant_angle = math.radians(orientation)
        self.info(f"spin_to_relative: orientation={orientation}")
        self.spin(spin_dist=radiant_angle, time_allowance=time_allowance)
        if wait:
            self.wait_until_task_complete()

    def go_to_absolute(self, *, x: float = None, y: float = None, orientation: float = None, wait: bool = True) -> None:
        goal = self._to_absolute_pose(x=x, y=y, orientation=orientation)
        self.info(f"go_to_absolute: x={goal.pose.position.x}, "
                  f"y={goal.pose.position.y}, orientation="
                  f"{self._quaternion_to_degree(goal.pose.orientation)}")
        self.goToPose(goal, behavior_tree=self.behavior_tree)
        if wait:
            self.wait_until_task_complete()

    def go_to_relative(self, *, x: float = None, y: float = None, orientation: float = None, wait: bool = True) -> None:
        goal = self._to_relative_pose(x=x, y=y, orientation=orientation)
        self.info(f"go_to_relative: x={x}, y={y}, orientation={orientation}")
        self.goToPose(goal, behavior_tree=self.behavior_tree)
        if wait:
            self.wait_until_task_complete()

    def go_through_absolute_poses(self, list_of_poses: list[dict], wait: bool = True) -> None:
        goal_poses = []
        for pose in list_of_poses:
            goal_poses.append(self._to_absolute_pose(**pose))
        self.info(f"go_through_absolute_poses: poses={list_of_poses}")
        # self.goThroughPoses(goal_poses)
        self.followWaypoints(goal_poses)
        if wait:
            self.wait_until_task_complete()

    def go_through_relative_poses(self, list_of_poses: list[dict], wait: bool = True) -> None:
        goal_poses = []
        for pose in list_of_poses:
            goal_poses.append(self._to_relative_pose(**pose))
        self.info(f"go_through_relative_poses: poses={list_of_poses}")
        # self.goThroughPoses(goal_poses)
        self.followWaypoints(goal_poses)
        if wait:
            self.wait_until_task_complete()

    def wait_until_task_complete(self) -> None:
        self.info("Waiting for task to complete")
        while not self.isTaskComplete():
            pass
        self.info("Task complete")

    def _to_absolute_pose(self, *, x: float = None, y: float = None, orientation: float = None) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        # Copy current pose to goal pose
        goal.pose = self.current_pose.pose.pose

        # Update goal pose with new position
        if x is not None:
            goal.pose.position.x = x
        if y is not None:
            goal.pose.position.y = y
        if orientation is not None:
            goal.pose.orientation = self._degree_to_quaternion(orientation)

        return deepcopy(goal)

    def _to_relative_pose(self, *, x: float = None, y: float = None, orientation: float = None) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        # Copy current pose to goal pose
        goal.pose = self.current_pose.pose.pose

        # Update goal pose with new position
        if x is not None:
            goal.pose.position.x += x
        if y is not None:
            goal.pose.position.y += y
        if orientation is not None:
            goal.pose.orientation = self._degree_to_quaternion(
                self._quaternion_to_degree(goal.pose.orientation) + orientation
            )

        return deepcopy(goal)

    def _current_pose_callback(self, msg: Odometry) -> None:
        self.current_pose = msg
        if not self.current_pose_received:
            self.info("Received first current pose")
            self.current_pose_received = True

    @staticmethod
    def _quaternion_to_degree(quaternion: Quaternion) -> float:
        # Calculate the rotation about the Z-axis
        theta_rad = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        )

        # Convert angle from radians to degrees
        theta_deg = math.degrees(theta_rad)

        return theta_deg

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
