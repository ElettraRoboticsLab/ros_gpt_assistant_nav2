import json
import rclpy

from std_msgs.msg import String

from .context_navigator import ContexNavigator
from .openai_assistant_parser import OpenAIAssistantParser


class Nav2RosGptAssistant(OpenAIAssistantParser):
    KNOWN_LOCATIONS = {
        "ZERO": {
            "key": "ZERO",  # make sure openai understands this key
            "name": "Initial position",
            "description": "Initial position of the robot in the map",
            "x": 0,
            "y": 0,
            "angle": 0,
        },
        "KITCHEN": {
            "key": "KITCHEN",  # make sure openai understands this key
            "name": "Kitchen",
            "description": "Just the kitchen",
            "x": 0.5,
            "y": 0.5,
            "angle": 30,
        },
        "BATHROOM": {
            "key": "BATHROOM",  # make sure openai understands this key
            "name": "Bathroom",
            "description": "Just the bathroom",
            "x": -1.5,
            "y": 0.5,
            "angle": 180,
        }
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.navigator = ContexNavigator()

        self.tts_publisher = self.create_publisher(String, '/tts', 10)

    def handle_dict_response(self, response: dict) -> None:
        """
        Override the default response handler to add custom handling.
        """
        if "tts" in response:
            msg = String()
            msg.data = json.dumps(response.get("tts", {}))
            self.tts_publisher.publish(msg)
        if "navigation" in response:
            self.handle_navigation_action(response.get("navigation", {}))

    def handle_navigation_action(self, navigation: dict) -> None:
        action = navigation.get("action", None)
        if action == "go_to_relative":
            self._info("Executing go_to_relative action")
            coordinates = navigation.get("coordinates", None)
            if not coordinates:
                self._error(f"Received coordinate is {coordinates}")
                return
            if type(coordinates) == list:
                coordinates = coordinates[0]
            self.navigator.go_to_relative(
                x=coordinates.get("x", None),
                y=coordinates.get("y", None),
                orientation=coordinates.get("angle", None),
            )

    def execute_tool_function(self, tool_call_id: str, tool_name: str) -> dict:
        """
        Override the default tool function to add custom functions.
        """
        self._info(f"Executing tool function {tool_name}")
        if tool_name == "get_known_locations":
            return {
                "tool_call_id": tool_call_id,
                "output": str(self.KNOWN_LOCATIONS)
            }
        return {
            "tool_call_id": tool_call_id,
            "output": f"Invalid output. {tool_name} is not a supported."
        }


def main(args=None):
    rclpy.init(args=args)
    node = Nav2RosGptAssistant()
    try:
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
