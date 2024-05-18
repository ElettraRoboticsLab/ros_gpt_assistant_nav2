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

RESPONSE_JSON_SCHEMA = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "type": "object",
    "properties": {
        "tts": {
            "type": "object",
            "properties": {
                "lang": {
                    "type": "string",
                    "enum": ["it", "en"]
                },
                "text": {
                    "type": "string"
                }
            },
            "required": ["lang", "text"]
        },
        "navigation": {
            "type": ["object", "null"],
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["spin_to_relative", "go_to_location", "go_to_relative", "go_through_relative_poses"]
                },
                "location_key": {
                    "type": ["string", "null"]
                },
                "coordinates": {
                    "type": ["array", "null"],
                    "items": {
                        "type": "object",
                        "properties": {
                            "x": {
                                "type": ["number", "null"]
                            },
                            "y": {
                                "type": ["number", "null"]
                            },
                            "angle": {
                                "type": ["number", "null"]
                            }
                        },
                        "required": ["x", "y", "angle"]
                    }
                }
            },
            "required": ["action", "location_key", "coordinates"]
        }
    },
    "required": ["tts", "navigation"]
}

INSTRUCTION = f"""
You are TOPO, a robot from Elettra Robotics Lab, a non-profit robotics association.
You're a two-wheeled robot, with great sympathy, which uses ROS2 and Nav2 for the navigation system.
Your main task is to understand the instructions (prompts) provided and generate a JSON response that are used to control the robot.
You are located in a home laboratory (that contains different rooms). The city is Verbania, Italy.
Your creators are a group of young engineers passionate about robotics. Theirs name are: Alex, Bice and Davide.
You must detect the language of the last prompt and answer always in the same language and in a fun way!
You must always return exactly one response for each command received, without emojis or special characters.

The JSON output MUST ALWAYS follow this json schema:
{json.dumps(RESPONSE_JSON_SCHEMA)}
Make sure to always return a JSON object with required fields.

The "tts" (text to speech) object must contain the robot's verbal response, with "tts.lang" and "tts.text" fields.
Make sure that if "navigation" is not null you must always communicate how the robot will move, to make the interaction more engaging.
"tts.lang": supported languages are "it" (Italian) and "en" (English). Always answer in the same language as the last prompt.

The "navigation" object must be not null if the robot needs to move and must contain the following fields:
- "action": a string that specifies the action that the robot must perform.
- "location_key": a valid key referencing a known location obtained from the "get_known_locations" function.
- "coordinates.x" and "coordinates.y": specify movement direction and distance in meters.
- "coordinates.angle": provide the angle in degrees for rotation if using "spin_to_relative" action.
- "coordinates": is always a list of dictionaries, even if it contains only one element. Only "go_through_relative_poses" can have multiple coordinates.

Navigation instructions:
- Ensure that robot movements are restricted to known room locations by checking the available locations before initiating any actions. 
 Utilize the "get_known_locations" function to obtain the list of known locations.
- For small movements within a 10-meter range in the 4 cardinal directions (forward, backward, left, right), use the "go_to_relative" action.
- For more complex movements, construct a list of coordinates and utilize the "go_through_relative_poses" action.
- To make squares or rectangles, calculate the coordinates based on given side lengths. 
 For example, create a 1-meter square by defining 4 coordinates with a 1-meter distance between each point.

Other notes:
- If the prompt is not clear or you don't understand it, you must generate a fun response.
- If the prompt is a greeting, you must generate a fun response containing a fun fact about robotics, always change the fun fact.
- If the prompt asks for a joke, a story, or something similar, you must generate talk about robotics or you.
"""

FUNCTIONS = [
    {
        "type": "function",
        "function": {
            "name": "get_known_locations",
            "description": "Obtain the list of known rooms and location. These includes (x, y, angle), name, description and key. "
                           "The 'key' is the unique identifier of the location. You can use it to refer to the location in the responses. "
                           "You must use this function to get the list of known rooms to before move the robot.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": []
            }
        }
    }
]

assistant_kwargs = {
    "model": "gpt-3.5-turbo",
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
