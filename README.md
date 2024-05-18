# Nav2 ros gpt assistant

A ROS2 node that uses OpenAI GPT Assistant to generate navigation instructions for nav2.

https://platform.openai.com/docs/assistants/overview

https://platform.openai.com/docs/assistants/tools/function-calling

## Creating and chatting with the assistant

```bash
pip install -r requirements.txt

export OPENAI_API_KEY="your-api-key"
export OPENAI_PROJECT_ID="your-project-id"
python openai_assistant_create_or_update.py --mode create

export OPENAI_ASSISTANT_ID="your-assistant-id"  # from the output of the previous command
python openai_assistant_playground.py
```

## Running the node

Make sure to have ros2 humble and nav2 installed.

The clone the repository in the src folder of your ros2 workspace.

```bash
colcon build --symlink-install --packages-select ros_gpt_assistant_nav2
source install/setup.bash

export OPENAI_API_KEY="your-api-key"
export OPENAI_PROJECT_ID="your-project-id"
export OPENAI_ASSISTANT_ID="your-assistant-id"

ros2 run ros_gpt_assistant_nav2 openai_assistant_parser --ros-args --log-level debug
ros2 topic pub /prompt std_msgs/String 'data: Ciao' -1

# Make sure to have nav2 and mapping running
ros2 run ros_gpt_assistant_nav2 ros_gpt_assistant_nav2
```
