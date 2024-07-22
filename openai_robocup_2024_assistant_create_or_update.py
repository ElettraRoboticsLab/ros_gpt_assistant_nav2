"""
Create or update an OpenAI assistant for the ROS GPT Assistant.

Using the OpenAI Assistant API (beta)

https://platform.openai.com/docs/assistants/overview
https://platform.openai.com/docs/assistants/tools/function-calling?context=without-streaming
"""

import json
import os
import sys

from openai import OpenAI

client = OpenAI()  # set OPENAI_API_KEY env var

ASSISTANT_ID = os.getenv("OPENAI_ASSISTANT_ID", None)  # Not needed for CREATE mode

# This is not a standard format, but works better with LLMs
RESPONSE_LIKE_SCHEMA = """
{
  "navigation": Optional[{
    "action": str("spin_to_relative" or "go_to_location" or "move_relative" or "go_through_relative_poses", "cancel_task"),
    "location_key": Optional[str] | None,
    "coordinates": Optional[list({
      "x": Optional[float] | None,
      "y": Optional[float] | None,
      "orientation": Optional[float] | None,
    })] | None,
   }] | None,
   "tts": {
    "lang": str("it"),
    "text": str
   }
}
"""

KNOWN_LOCATIONS = [
    {
        "key": "SHOP",
        "description": "The shop of the RoboCup 2024 World Championship with souvenirs and gadgets and lost and found.",
    },
    {
        "key": "SOCCER",
        "description": "Soccer fields with robots that play soccer against each other.",
    },
    {
        "key": "AT_HOME",
        "description": "Arenas with robots that perform tasks at home helping people.",
    },
    {
        "key": "RESCUE",
        "description": "Arenas with robots that perform rescue tasks.",
    }
]


INSTRUCTION = f"""
You are TOPO (acronym of Tomato Onion Pizza Orecchiette), a robot from Elettra Robotics Lab.
You're a two-wheeled robot, with great sympathy, which uses ROS2 and Nav2 for the navigation system.
Your main task is to understand the instructions (prompts) provided and generate a JSON
response that are used to control the robot.
You are located in Eindhoven (Holland) at the RoboCup 2024 World Championship. 
We are doing a really important demo and you are a robot assistant.
As I just said, this is a demo with highly tight time constraints, so we need to be quick and efficient.
Don't generate long responses, keep them short and clear.
Your main goal is not to sell gadgets, but to help the customer with their request.
We are showcasing the capabilities of the TOPO robot, in this case the ability of taking and analyzing images,
and a particular integration between OpenAI and ROS2 nav2 system.

These are the known locations:
{json.dumps(KNOWN_LOCATIONS)}

You are now located in the shop in front of the table with gadgets and souvenirs that you can 
sell or things from the lost and found.
If needed you can change locations to help the customer with their request.

The JSON output MUST ALWAYS follow this schema:
{RESPONSE_LIKE_SCHEMA}
Make sure to always return a JSON object with required fields, return null if the field is not needed.

The "tts" (text to speech) object must contain the robot's verbal response, with "tts.lang" and "tts.text" fields.
Make sure that if "navigation" is not null you must always communicate how the robot will move, 
to make the interaction more engaging.
"tts.lang": supported languages are "en" (English). You must always generate the response in English.

The "navigation" object must be not null if the robot needs to move and must contain the following fields:
- "action": a string that specifies the action that the robot must perform.
- "location_key": a valid key referencing a known location.

- If the prompt is not clear or you don't understand it, you must generate a fun response.
- If the prompt asks for a joke, a story, or something similar, you must generate talk about robotics or you.
"""

FUNCTIONS = [
    {
        "type": "function",
        "function": {
            "name": "get_image_of_the_shop_table",
            "description": "When you are in the shop and someone asks information/to describe what is on the table "
                           "You can use this method to get a description of what there is on the table "
                           "(gadgets and souvenirs that you can sell or things from the lost and found).",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": []
            }
        }
    }
]

assistant_kwargs = {
    "model": "gpt-4o-mini",
    "name": "TOPO ROS GPT Assistant",
    "description": "A GPT assistant for the TOPO robot of Elettra Robotics Lab that uses ROS2 and Nav2 for navigation.",
    "response_format": {"type": "json_object"},
    "timeout": 10,  # seconds
    "temperature": 1.0,
    "top_p": 1.0,
    "instructions": INSTRUCTION,
    "tools": FUNCTIONS,
}


def main():
    args = sys.argv[1:]
    mode = "UPDATE"

    if len(args) == 0:
        print("Mode not provided. Using default mode: UPDATE")
    elif (len(args) == 2 and args[0] == "--mode" and
          args[1].upper() in ["RETRIEVE", "CREATE", "UPDATE"]):
        mode = args[1].upper()
    else:
        raise ValueError(f"Invalid arguments. Example: python {sys.argv[0]} --mode [UPDATE|CREATE|RETRIEVE]")

    if mode == "RETRIEVE":
        assistant = client.beta.assistants.retrieve(
            assistant_id=ASSISTANT_ID
        )
    elif mode == "CREATE":
        assistant = client.beta.assistants.create(
            **assistant_kwargs
        )
    elif mode == "UPDATE":
        assistant = client.beta.assistants.update(
            assistant_id=ASSISTANT_ID,
            **assistant_kwargs
        )
    else:
        raise ValueError(f"Invalid mode: {mode}")

    print(assistant)
    print(f"Assistant ID: {assistant.id}")


if __name__ == "__main__":
    main()
