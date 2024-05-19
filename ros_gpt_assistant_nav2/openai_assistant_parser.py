import json
import os
import traceback

import rclpy
from openai import OpenAI
from openai.types.beta.threads.run import Run
from rclpy.node import Node
from std_msgs.msg import String


class OpenAIAssistantParser(Node):

    def __init__(self, *args, **kwargs):
        super().__init__("openai_assistant_parser")

        self.prompt_topic = self.declare_parameter("prompt_topic", "/prompt").value

        self.client = OpenAI()  # set OPENAI_API_KEY and OPENAI_PROJECT_ID env vars
        self.assistant_id = os.environ.get("OPENAI_ASSISTANT_ID")
        self.thread = self.client.beta.threads.create()

        self.prompt_sub = self.create_subscription(
            String,
            self.prompt_topic,
            self._prompt_callback,
            10,
        )

        self.openai_status_publisher = self.create_publisher(String, '/openai_status', 10)
        self.openai_output_publisher = self.create_publisher(String, '/openai_output', 10)

        self._info(f"OpenAI Assistant Prompt Parser node ready and "
                   f"waiting for prompts on {self.prompt_topic} topic.")

    def _prompt_callback(self, msg: String) -> None:
        """
        Callback for the topic subscription.
        """
        prompt = msg.data
        self._info(f"Received prompt data: {prompt}")
        if not prompt:
            self._error("Empty prompt")
            return

        try:
            response = self._parse_prompt(prompt)
        except Exception:
            self._error(traceback.format_exc())
            response = self._error_response("Exception occurred during parsing prompt. "
                                            "Check logs for details.")

        self._info(f"Response: {response}")
        self.handle_dict_response(response)

    def handle_dict_response(self, response: dict) -> None:
        """
        Handle the response from the OpenAI Assistant.
        Override this method to customize the response handling.
        Expected minimum response format:
        {
            "tts": {
                "lang": "en",
                "text": "Text to speech output"
            }
        }
        """
        pass

    def _update_topic_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.openai_status_publisher.publish(msg)

    def _update_topic_output(self, output: dict) -> None:
        msg = String()
        msg.data = json.dumps(output)
        self.openai_output_publisher.publish(msg)

    def _parse_prompt(self, prompt: str) -> dict:
        """
        Parse the prompt and send it to the OpenAI Assistant.
        """
        self._update_topic_status("sending")
        self._debug("Sending prompt to OpenAI Assistant...")
        self.client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=prompt,
        )
        self._update_topic_status("waiting")
        self._info("Waiting for response...")
        run = self.client.beta.threads.runs.create_and_poll(
            thread_id=self.thread.id,
            assistant_id=self.assistant_id,
        )
        self._debug("Parsing run...")
        # self._debug(f"Run: {run}")
        out = self._parse_run(run)
        self._update_topic_output(out)
        self._update_topic_status("completed")
        return out

    def _parse_run(self, run: Run) -> dict:
        """
        Parse the run response from the OpenAI Assistant.
        It may be a completed run or a run that requires action.
        If it requires action, the tool outputs are submitted and the run is parsed again.
        """
        if run.status == "completed":
            self._debug("Run completed")
            received_message = None
            try:
                received_message = self._get_last_thread_response()
                response = json.loads(received_message)
                return response
            except ValueError as e:
                self._error(f"received_message: {received_message}")
                return self._error_response(f"Failed to parse JSON response. {e}")

        elif run.status == "requires_action":
            self._info("Run requires action")
            tool_outputs = []

            # Loop through each tool in the required action section
            for tool in run.required_action.submit_tool_outputs.tool_calls:
                tool_outputs.append(
                    self.execute_tool_function(
                        tool_call_id=tool.id,
                        tool_name=tool.function.name
                    )
                )
            self._debug("Actions executed")

            # Submit the tool outputs
            run2 = self.client.beta.threads.runs.submit_tool_outputs_and_poll(
                thread_id=self.thread.id,
                run_id=run.id,
                tool_outputs=tool_outputs
            )
            self._debug("Tool outputs submitted successfully")

            # After submitting the tool outputs, parse the run again
            return self._parse_run(run2)

        else:
            self._error(f"Run error output: {run}")
            return self._error_response(f"Run status is: {run.status}")

    def execute_tool_function(self, tool_call_id: str, tool_name: str) -> dict:
        """
        Execute the tool function based on the tool name.
        Override this method to add more tool functions.
        """
        self._error(f"Executing tool function {tool_name}. "
                    f"Override 'execute_tool_function' method to add your functions.")
        return {
            "tool_call_id": tool_call_id,
            "output": f"Invalid output. {tool_name} is not a supported."
        }

    def _get_last_thread_response(self) -> str:
        """
        Get the last response from the thread.
        """
        messages = self.client.beta.threads.messages.list(
            thread_id=self.thread.id
        )
        return messages.data[0].content[0].text.value

    def _debug(self, msg: str) -> None:
        self.get_logger().debug(msg)

    def _info(self, msg: str) -> None:
        self.get_logger().info(msg)

    def _error(self, msg: str) -> None:
        self.get_logger().error(msg)

    def _error_response(self, msg: str) -> dict:
        self._error(msg)
        return {
            "error": True,
            "tts": {
                "lang": "en",
                "text": f"I've got a problem. {msg}",
            }
        }


def main(args=None):
    rclpy.init(args=args)
    node = OpenAIAssistantParser()
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
