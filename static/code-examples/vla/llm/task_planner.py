#!/usr/bin/env python3
"""
LLM-Based Task Planner for Robotics

Use large language models to decompose high-level tasks into robot actions.

Dependencies: openai, rclpy
"""

import openai
import json
import os

class LLMTaskPlanner:
    """
    Convert natural language commands to structured robot action plans.
    """

    def __init__(self, api_key=None):
        """
        Initialize LLM planner.

        Args:
            api_key (str): OpenAI API key (or set OPENAI_API_KEY env var)
        """
        if api_key:
            openai.api_key = api_key
        else:
            openai.api_key = os.getenv("OPENAI_API_KEY")

        self.system_prompt = """
You are a robot task planner. Convert natural language commands into structured action sequences.

Available robot actions:
- navigate_to(location): Move to specified location
- pick_object(object_name): Grasp an object
- place_object(location): Place held object
- open_door(): Open a door
- say(text): Speak text

Output format (JSON):
{
  "task": "original command",
  "actions": [
    {"action": "action_name", "params": {...}},
    ...
  ]
}
"""

    def plan_task(self, command):
        """
        Generate action plan from natural language command.

        Args:
            command (str): Natural language command

        Returns:
            dict: Structured action plan
        """
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": f"Command: {command}"}
                ],
                temperature=0.3,
                max_tokens=500
            )

            plan_text = response.choices[0].message.content
            plan = json.loads(plan_text)

            return plan

        except Exception as e:
            print(f"Error in task planning: {e}")
            return None

    def execute_plan(self, plan):
        """
        Execute planned actions (placeholder - integrate with ROS 2).

        Args:
            plan (dict): Action plan
        """
        print(f"\nExecuting plan for: {plan['task']}")
        print(f"Number of actions: {len(plan['actions'])}\n")

        for i, action in enumerate(plan['actions'], 1):
            print(f"Step {i}: {action['action']}")
            print(f"  Parameters: {action.get('params', {})}")

            # TODO: Map to actual ROS 2 actions
            # Example:
            # if action['action'] == 'navigate_to':
            #     self.send_navigation_goal(action['params']['location'])

def main():
    """
    Demo: Task planning from voice command.
    """
    planner = LLMTaskPlanner()

    # Example commands
    commands = [
        "Go to the kitchen and bring me a cup",
        "Open the door and say hello",
        "Pick up the red ball and place it in the box"
    ]

    for command in commands:
        print(f"\n{'='*60}")
        print(f"Command: {command}")
        print('='*60)

        plan = planner.plan_task(command)

        if plan:
            planner.execute_plan(plan)
        else:
            print("Failed to generate plan")

if __name__ == "__main__":
    main()
