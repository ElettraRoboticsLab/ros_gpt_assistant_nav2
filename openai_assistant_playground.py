"""
Chatting with the assistant (playground).
You must create the assistant before running this script.
"""

import json
import os

from openai import OpenAI
from openai.types.beta.threads.run import Run

client = OpenAI()  # set OPENAI_API_KEY env var

ASSISTANT_ID = os.getenv("OPENAI_ASSISTANT_ID")


def get_last_thread_response(thread_id: str) -> str:
    """
    Get the last response from the thread.
    """
    messages = client.beta.threads.messages.list(
        thread_id=thread_id
    )
    return messages.data[0].content[0].text.value


def parse_run(thread_id: str, run: Run) -> str:
    """
    Parse the run response from the OpenAI Assistant.
    It may be a completed run or a run that requires action.
    If it requires action, the tool outputs are submitted and the run is parsed again.
    """
    if run.status == "completed":
        response = get_last_thread_response(thread_id)
        return response

    elif run.status == "requires_action":
        print("Assistant: [requires action]")
        tool_outputs = []
        for tool in run.required_action.submit_tool_outputs.tool_calls:
            print(f"\tAction required: {tool.function.name} - skipping execution in this playground.")
            tool_outputs.append({
                "tool_call_id": tool.id,
                "output": "Action skipped because this is a playground. Act like you received the output."  # magic
            })

        # Submit the tool outputs
        run2 = client.beta.threads.runs.submit_tool_outputs_and_poll(
            thread_id=thread_id,
            run_id=run.id,
            tool_outputs=tool_outputs
        )
        return parse_run(thread_id, run2)

    else:
        print(f"Assistant: [error] {run}")
        return f"[error status]: {run.status}"


def send_message(thread_id: str, message: str) -> str:
    message = client.beta.threads.messages.create(
        thread_id=thread_id,
        role="user",
        content=message,
    )

    run = client.beta.threads.runs.create_and_poll(
        thread_id=thread_id,
        assistant_id=ASSISTANT_ID,
    )

    return parse_run(thread_id, run)


def main():
    assistant = client.beta.assistants.retrieve(assistant_id=ASSISTANT_ID)

    print(assistant)
    print(f'Assistant "{assistant.name}" loaded (id:{assistant.id})')

    thread = client.beta.threads.create()

    while True:
        message = input("\nYou: ")
        if not message:
            continue
        response = send_message(thread.id, message)
        response_string = json.dumps(json.loads(response), indent=2, ensure_ascii=False)
        print(f"Assistant: {response_string}")


if __name__ == "__main__":
    main()
