# Data Model: Vision-Language-Action (VLA) Module

This document describes the data model for the VLA module.

## Entities

### Command

Represents a user's instruction to the robot.

- **Attributes**:
    - `text` (string): The raw text of the user's command.
    - `action` (string): The parsed action to be performed (e.g., "navigate", "pick").
    - `target` (string): The object or location that is the target of the action.
    - `location` (string, optional): A secondary location, if applicable (e.g., "the kitchen sink" in "put the cup in the kitchen sink").

### Plan

A sequence of steps to be executed by the robot to fulfill a command.

- **Attributes**:
    - `goal` (string): The overall goal of the plan (e.g., "clean_room").
    - `steps` (array of strings): A list of actions to be performed in sequence (e.g., `["scan_room", "identify_dirty_objects", "navigate_to_object", "pick_object", "place_in_bin"]`).

### Object

A physical item in the environment that the robot can perceive and interact with.

- **Attributes**:
    - `id` (integer): A unique identifier for the object.
    - `name` (string): The name of the object (e.g., "red cup").
    - `position` (Point): The 3D position of the object in the world.
    - `orientation` (Quaternion): The orientation of the object in the world.
    - `bounding_box` (BoundingBox): The bounding box of the object.

## State Transitions

The robot will have a state machine to track its current state. The states will include:

- `IDLE`: Waiting for a command.
- `LISTENING`: Capturing audio.
- `PROCESSING`: Converting speech to text and generating a plan.
- `EXECUTING`: Executing the plan.
- `COMPLETED`: The plan has been successfully executed.
- `FAILED`: The plan has failed.
