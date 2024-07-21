import json
import os
import time

import rclpy
import requests
from cv_bridge import CvBridgeError
from ros_gpt_assistant_nav2.openai_assistant_parser import OpenAIAssistantParserNode
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Nav2RosGptAssistant(OpenAIAssistantParserNode):
    KNOWN_LOCATIONS = {
        "ZERO": {
            "x": 0,
            "y": 0,
            "orientation": 0,
        },
        "KITCHEN": {
            "x": 0.5,
            "y": 0.5,
            "orientation": 30,
        },
        "BATHROOM": {
            "x": -1.5,
            "y": 0.5,
            "orientation": 180,
        },
        "LAB_ZERO": {
            "x": 0,
            "y": 0,
            "orientation": 0,
        }
    }

    def __init__(self, *args, **kwargs):
        super().__init__("ros_gpt_assistant_nav2", *args, **kwargs)

        self.tts_topic = self.declare_parameter("tts_topic", "/tts").value
        self.enable_navigation = self.declare_parameter("enable_navigation", True).value

        self.navigator = None
        if self.enable_navigation:
            from ros_gpt_assistant_nav2.context_navigator import ContextNavigator

            self.navigator = ContextNavigator()

        self.tts_publisher = self.create_publisher(String, self.tts_topic, 10)

        self.gpt_image_key = os.getenv("OPENAI_TASK2")
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback_camera,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        self.last_img = None

    def image_callback_camera(self, data):
        # self.get_logger().info('Received an image')
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv2 error {e}")

    def handle_dict_response(self, response: dict) -> None:
        """
        Override the default response handler to add custom handling.
        """
        if "tts" in response:
            self.handle_tts_action(response.get("tts", {}))
        if "navigation" in response:
            if self.navigator is None:
                self._error("Navigation is not enabled.")
            else:
                self.handle_navigation_action(response.get("navigation", {}))

    def handle_tts_action(self, tts: dict) -> None:
        """
        Redirect the TTS action from the response to the TTS topic.
        """
        if tts is None or not tts:
            return
        msg = String()
        msg.data = json.dumps(tts)
        self.tts_publisher.publish(msg)

    def handle_navigation_action(self, navigation: dict) -> None:
        """
        Handle the navigation action from the response.
        This dict is generated from an LLM model, so it is not guaranteed to have all keys.
        """
        if navigation is None or not navigation:
            return

        action = navigation.get("action", None)

        if action == "spin_to_relative":
            self._info("Executing spin_to_relative action")
            coordinate = navigation.get("coordinates", [{}])[0]
            if not coordinate:
                self._error(f"Received coordinate is {navigation.get('coordinates', None)}")
                return
            self.navigator.spin_to_relative(
                orientation=float(coordinate.get("orientation", None) or 0.0),
            )

        if action == "go_to_location":
            self._info("Executing go_to_location action")
            location_key = navigation.get("location_key", None)
            if location_key is None:
                self._error("Received no location_key")
                return
            coordinate = self.KNOWN_LOCATIONS.get(location_key, None)
            if coordinate is None:
                self._error(f"Invalid location_key: {location_key}")
                return
            self.navigator.go_to_absolute(
                x=float(coordinate.get("x", None) or 0.0),
                y=float(coordinate.get("y", None) or 0.0),
                orientation=float(coordinate.get("orientation", None) or 0.0),
            )

        if action == "move_relative":
            self._info("Executing move_relative action")
            coordinate = navigation.get("coordinates", [{}])[0]
            if not coordinate:
                self._error(f"Received coordinate is {navigation.get('coordinates', None)}")
                return
            self.navigator.move_relative(
                x=float(coordinate.get("x", None) or 0.0),
                y=float(coordinate.get("y", None) or 0.0),
                orientation=float(coordinate.get("orientation", None) or 0.0),
            )

        if action == "go_through_relative_poses":
            self._info("Executing go_through_relative_poses action")
            coordinates = navigation.get("coordinates", [])
            if not coordinates:
                self._error(f"Received coordinates is {navigation.get('coordinates', None)}")
                return
            self.navigator.go_through_relative_poses(
                list_of_poses=coordinates,
            )

        if action == "cancel_task":
            self._info("Executing cancel_task action")
            self.navigator.cancel_task()

    def get_characteristics(self):
        # OpenAI API Key

        prompt = """
          You will get an image of a shop table, there will be some items on it.
          You need to describe the items on the table.
          The items could be some souvenirs, gadgets, or lost and found objects.
          """

        # Getting the base64 string
        # while last_img return None call it
        while self.last_img is None:
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.5)

        base64_image = self.encode_image(self.last_img)
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }

        payload = {
            "model": "gpt-4o-mini",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": prompt
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            }
                        }
                    ]
                }
            ],
            "max_tokens": 800
        }

        response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
        self.log(f"open ai response={response.json()}")
        return response.json()["choices"][0]["message"]["content"]

    def execute_tool_function(self, tool_call_id: str, tool_name: str) -> dict:
        """
        Override the default tool function to add custom functions.
        """
        self._info(f"Executing tool function {tool_name}")
        if tool_name == "get_image_of_the_shop_table":
            return {
                "tool_call_id": tool_call_id,
                "output": str(self.get_characteristics())
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


if __name__ == "__main__":
    main()
