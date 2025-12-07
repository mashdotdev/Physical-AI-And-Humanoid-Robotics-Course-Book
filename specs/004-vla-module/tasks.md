# Actionable Tasks: Vision-Language-Action (VLA) Module

**Branch**: `004-vla-module` | **Date**: 2025-12-07 | **Spec**: [link:./spec.md] | **Plan**: [link:./plan.md]

This document breaks down the implementation of the VLA module into actionable tasks, organized by user story.

## Phase 1: Setup

- [X] T001 Create a new ROS2 package named `ros2_vla` in the `src` directory.
- [X] T002 Add the required dependencies to `package.xml` (e.g., `rclpy`, `std_msgs`, `sensor_msgs`, `vla_msgs`).
- [X] T003 Create the `launch` and `nodes` directories inside `src/ros2_vla`.
- [X] T004 Copy the message and service definitions from `specs/004-vla-module/contracts` to `src/ros2_vla/msg` and `src/ros2_vla/srv`.
- [X] T005 Create the action definition from `specs/004-vla-module/contracts` to `src/ros2_vla/action`.


## Phase 2: Foundational Tasks

- [X] T006 [P] Create a ROS2 node `audio_capture_node.py` in `src/ros2_vla/nodes` that captures audio from a microphone and publishes it to the `/audio_capture/audio` topic.
- [X] T007 [P] Create a ROS2 node `whisper_node.py` in `src/ros2_vla/nodes` that subscribes to the `/audio_capture/audio` topic, sends the audio to the Whisper ASR service, and publishes the recognized text to the `/speech_to_text/text` topic.

## Phase 3: User Story 1 - Voice-to-Action

**Goal**: A user gives a voice command to the robot, and the robot executes the corresponding action.

**Independent Test**: The user can say "pick up the red box" and the robot will move to the red box, pick it up, and report completion.

- [X] T008 [US1] Create a ROS2 node `llm_node.py` in `src/ros2_vla/nodes` that subscribes to the `/speech_to_text/text` topic.
- [X] T009 [US1] Implement the `GetPlan` service in the `llm_node.py` that sends the recognized text to GPT-4o and parses the response to generate a `Plan` message.
- [X] T010 [US1] Create a ROS2 node `task_dispatcher_node.py` in `src/ros2_vla/nodes` that calls the `GetPlan` service to get a plan.
- [X] T011 [US1] Implement the logic in `task_dispatcher_node.py` to call the appropriate action servers based on the plan (e.g., `/nav2/navigate_to_pose`, `/manipulation/pick_and_place`).

## Phase 4: User Story 2 - Cognitive Planning

**Goal**: A user gives a complex, high-level voice command, and the robot breaks it down into a sequence of actions and executes them.

**Independent Test**: The user can say "clean the room" and the robot will scan the room, identify objects to be cleaned, and move them to a designated bin.

- [X] T012 [US2] Extend the prompt for GPT-4o in `llm_node.py` to handle complex commands and generate a sequence of steps in the `Plan` message.
- [X] T013 [US2] Implement the `ExecutePlan` action server in `task_dispatcher_node.py` that executes the steps of the plan in sequence.
- [X] T014 [US2] Integrate with the perception stack to get the list of objects in the room.

## Phase 5: User Story 3 - Capstone Project

**Goal**: A user gives a multi-step command that involves navigation, perception, and manipulation.

**Independent Test**: The user can say "Pick up the cup from the table and put it in the kitchen sink" and the robot will perform the entire sequence of actions.

- [X] T015 [US3] Create a launch file `vla.launch.py` in `src/ros2_vla/launch` that starts all the required nodes for the VLA module.
- [X] T016 [US3] Create a test script that sends a voice command and verifies that the robot completes the task successfully.

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T017 Implement voice feedback for task completion, clarification, and failure.
- [X] T018 Implement robust error handling and recovery mechanisms.
- [X] T019 Write unit and integration tests for all the nodes.

## Dependencies

- User Story 2 depends on the completion of User Story 1.
- User Story 3 depends on the completion of User Story 2.

## Parallel Execution

- T006 and T007 can be implemented in parallel.
- Within each user story, the node creation can be done in parallel, but the integration will be sequential.
