import curses

import rclpy
from rclpy.node import Node
from ros_gpt_assistant_nav2.context_navigator import ContextNavigator


class TeleopNavigator(Node):
    XY_STEP = 0.5
    ORIENTATION_STEP = 15.0

    def __init__(self):
        super().__init__('teleop_navigator')
        self.navigator = ContextNavigator()
        self.screen = curses.initscr()
        curses.cbreak()
        self.screen.keypad(True)
        self.screen.timeout(100)

    def destroy_node(self):
        curses.nocbreak()
        self.screen.keypad(False)
        curses.echo()
        curses.endwin()
        self.navigator.destroy_node()
        super().destroy_node()

    def screen_reset(self):
        self.screen.clear()
        self.screen.addstr(0, 0, "Teleop Nav2 Navigator")
        self.screen.addstr(1, 0, f"x,y step: {self.XY_STEP}, orientation step: {self.ORIENTATION_STEP}")
        self.screen.addstr(2, 0, "Use arrow keys to move, q to quit")
        self.screen.addstr(3, 0, "w/s: move_relative, a/d: spin_to_relative")
        self.screen.addstr(4, 0, "g: go_to_absolute, r: go_to_relative")
        self.screen.addstr(5, 0, "t: move_relative with input values")
        self.screen.addstr(7, 0, f"Current pose: {self.navigator.get_current_pose_tuple()}")

    def run(self):
        while rclpy.ok():
            self.screen_reset()
            key = self.screen.getch()
            if key == ord('q'):
                break
            elif key == curses.KEY_UP or key == ord('w'):
                self.navigator.move_relative(x=self.XY_STEP, wait=False)
            elif key == curses.KEY_DOWN or key == ord('s'):
                self.navigator.move_relative(x=-self.XY_STEP, wait=False)
            elif key == curses.KEY_LEFT or key == ord('a'):
                self.navigator.spin_to_relative(orientation=self.ORIENTATION_STEP, wait=False)
            elif key == curses.KEY_RIGHT or key == ord('d'):
                self.navigator.spin_to_relative(orientation=-self.ORIENTATION_STEP, wait=False)
            elif key == ord('g'):
                self.go_to_absolute_with_input()
            elif key == ord('r'):
                self.go_to_relative_with_input()
            elif key == ord('t'):
                self.move_relative_with_input()

            self.navigator.spin_once()
            rclpy.spin_once(self, timeout_sec=0.1)

    def go_to_absolute_with_input(self):
        x, y, orientation = self._get_pose_input()
        self.navigator.go_to_absolute(x=x, y=y, orientation=orientation, wait=False)

    def go_to_relative_with_input(self):
        x, y, orientation = self._get_pose_input()
        self.navigator.go_to_relative(x=x, y=y, orientation=orientation, wait=False)

    def move_relative_with_input(self):
        x, y, orientation = self._get_pose_input()
        self.navigator.move_relative(x=x, y=y, orientation=orientation, wait=False)

    def _get_pose_input(self):
        self.screen.addstr(8, 0, "Enter x, y, orientation:")
        curses.nocbreak()
        self.screen.keypad(False)
        curses.echo()
        curses.endwin()
        x = input("Enter x: ")
        y = input("Enter y: ")
        orientation = input("Enter orientation: ")
        curses.cbreak()
        curses.noecho()
        return float(x), float(y), float(orientation)


def main(args=None):
    rclpy.init(args=args)
    teleop_navigator = TeleopNavigator()
    try:
        teleop_navigator.run()
    except Exception as e:
        teleop_navigator.get_logger().error(f"Exception: {str(e)}")
    finally:
        teleop_navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
