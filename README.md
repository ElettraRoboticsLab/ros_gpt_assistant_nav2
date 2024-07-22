# Nav2 ros gpt assistant

A ROS2 node that uses OpenAI GPT Assistant to generate navigation instructions for nav2.

https://platform.openai.com/docs/assistants/overview

https://platform.openai.com/docs/assistants/tools/function-calling

## How it works

This node expects a string message on the `/prompt` topic.

A TTS (Text To Speech) output is published on the `/tts` topic.

The node interacts with NAV2 by publishing goal poses on the `/goal_pose` topic, extending the BasicNavigaror nav2
class.

If needed by the context, it can automatically "Take a photo" from the topic `/camera/image_raw` and send it as a prompt
to the assistant.

## Creating and chatting with the assistant

This is useful to test the assistant without the need of ROS2.

Set up the OpenAI API key and project id in the environment variables.
Then run the following commands to create the assistant and chat with it.

```bash
pip install -r requirements.txt

export OPENAI_API_KEY="your-api-key"
export OPENAI_PROJECT_ID="your-project-id"
python openai_assistant_create_or_update.py --mode create

export OPENAI_ASSISTANT_ID="your-assistant-id"  # from the output of the previous command
python openai_assistant_playground.py
```

## Running the node in ROS2

Make sure to have ROS2 Humble and nav2 installed.

Then clone the repository in the src folder of your ROS2 workspace.

```bash
colcon build --symlink-install --packages-select ros_gpt_assistant_nav2
source install/setup.bash

export OPENAI_API_KEY="your-api-key"
export OPENAI_PROJECT_ID="your-project-id"
export OPENAI_ASSISTANT_ID="your-assistant-id"

# Make sure to have nav2 and mapping running
ros2 run ros_gpt_assistant_nav2 ros_gpt_assistant_nav2
ros2 topic pub /prompt std_msgs/String 'data: Ciao' -1
```
