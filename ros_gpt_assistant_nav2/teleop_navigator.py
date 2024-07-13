import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from ros_gpt_assistant_nav2.context_navigator import ContextNavigator


class TeleopNavigator(Node):
    """
    Helper node to test ContextNavigator using keyboard input.
    """
    XY_STEP = 0.5
    ORIENTATION_STEP = 15.0

    def __init__(self):
        super().__init__("teleop_navigator")
        self.navigator = ContextNavigator()

    def destroy_node(self):
        self.navigator.destroy_node()
        super().destroy_node()

    def screen_reset(self):
        print("\033[H\033[J", end='')  # Clear screen
        print("Teleop Nav2 Navigator")
        print(f"x,y step: {self.XY_STEP}, orientation step: {self.ORIENTATION_STEP}")
        print("Use arrow keys to move, q to quit")
        print("w/s: move_relative, a/d: spin_to_relative")
        print("g: go_to_absolute, r: go_to_relative")
        print("t: move_relative with input values")
        print("c: cancel nav2 goal")

    def run(self):
        self.screen_reset()
        while rclpy.ok():
            # print("Enter command: ", end='', flush=True)
            key = self.get_key_with_timeout(0.20)
            if key != "":
                self.clear_log_space()
                print(f"Key pressed: '{key}'")
            if key == "q":
                break
            elif key == "w":
                self.navigator.move_relative(x=self.XY_STEP, wait=False)
            elif key == "s":
                self.navigator.move_relative(x=-self.XY_STEP, wait=False)
            elif key == "a":
                self.navigator.spin_to_relative(orientation=self.ORIENTATION_STEP, wait=False)
            elif key == "d":
                self.navigator.spin_to_relative(orientation=-self.ORIENTATION_STEP, wait=False)
            elif key == "g":
                self.go_to_absolute_with_input()
            elif key == "r":
                self.go_to_relative_with_input()
            elif key == "t":
                self.move_relative_with_input()
            elif key == "c":
                self.navigator.cancel_task()

            self.navigator.spin_once()
            rclpy.spin_once(self, timeout_sec=0.1)
            self.update_screen()

    def update_screen(self):
        # Move cursor to the top left and reprint the current pose
        print("\033[H", end='')
        print("\033[10B", end='')
        print(f"Current pose: {self.navigator.get_current_pose_tuple()}                 ")

    @staticmethod
    def clear_log_space():
        # Clear the logs - I know it's the better way :)
        print("\033[H", end='')
        print("\033[11B", end='')
        for _ in range(10):
            out = " " * 160
            print(out)
        print("\033[H", end='')
        print("\033[12B", end='')

    @staticmethod
    def get_key_with_timeout(timeout):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
            else:
                return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def go_to_absolute_with_input(self):
        x, y, orientation = self._get_pose_input()
        self.navigator.go_to_absolute(x=x, y=y, orientation=orientation, wait=False)

    def go_to_relative_with_input(self):
        x, y, orientation = self._get_pose_input()
        self.navigator.go_to_relative(x=x, y=y, orientation=orientation, wait=False)

    def move_relative_with_input(self):
        x, y, orientation = self._get_pose_input()
        self.navigator.move_relative(x=x, y=y, orientation=orientation, wait=False)

    @staticmethod
    def _get_pose_input():
        x = input("Enter x: ")
        y = input("Enter y: ")
        orientation = input("Enter orientation: ")
        return float(x), float(y), float(orientation)


def main(args=None):
    rclpy.init(args=args)
    teleop_navigator = TeleopNavigator()
    try:
        teleop_navigator.run()
    except Exception as e:
        teleop_navigator.get_logger().error(f"Exception: {str(e)}")
    except KeyboardInterrupt:
        pass
    finally:
        teleop_navigator.destroy_node()


if __name__ == "__main__":
    main()
