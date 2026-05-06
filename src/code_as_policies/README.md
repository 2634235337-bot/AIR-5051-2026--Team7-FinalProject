# Code as Policies for Sagittarius

This directory contains the language-to-robot layer for the Sagittarius real-robot adaptation of the Code as Policies project.

The code here is responsible for turning natural-language instructions into executable robot behavior. At a high level, the flow is:

`user instruction -> LMP code generation in the notebook -> environment wrapper -> Sagittarius ROS backend`

## What Each File Does

### `Interactive_Demo.ipynb`

This is the main entry point.

It contains:

- The Language Model Program (LMP) runtime.
- Prompt templates and few-shot examples for code generation.
- Helper parsers such as object-name parsing, position parsing, and question parsing.
- The real-robot interactive workflow for entering an API key, initializing the robot environment, scanning the workspace, and running natural-language commands.

In practice, this notebook is the place where a user types commands like "put the blue block in area A", and the model generates Python policy code that calls the environment APIs.

### `sagittarius_env.py`

This file is the bridge between the notebook and the ROS workspace in `sagittarius_ws/`.

Its main responsibilities are:

- Connecting to the Sagittarius action server.
- Reading camera images from ROS topics.
- Detecting colored blocks from the camera stream.
- Converting image coordinates into robot workspace coordinates.
- Maintaining an in-memory `world_state` for visible objects and named positions.
- Executing robot actions such as move, pick, and place.
- Retrying failed picks after refreshing perception when needed.

If `Interactive_Demo.ipynb` is the reasoning layer, `sagittarius_env.py` is the execution and perception layer.

### `environment.interactive_demo.yml`

This is the Conda environment definition for the interactive demo.

It describes the Python packages needed to run the notebook-side code, including numerical libraries, geometry tools, OpenCV, Jupyter, and the legacy OpenAI SDK version used by the notebook.

### `REAL_ROBOT_TEST_FLOW.md`

This file is an operator-facing runbook for real hardware.

It explains how to:

- Build the ROS workspace.
- Launch MoveIt, the Sagittarius control action server, and the camera.
- Verify backend connectivity.
- Run smoke tests.
- Start the notebook and validate the full interaction path.

Use this file when bringing the full real-robot stack online.

### `scripts/`

This folder contains small operational utilities for the real-robot workflow:

- `check_ws_backend.sh`: checks whether the required ROS topics and services are available.
- `sgr_ctrl_smoke_test.py`: sends a simple motion command to verify the action backend.
- `cancel_sgr_goal.sh`: cancels an active Sagittarius action goal.

These scripts are mainly for debugging and validation, not for high-level task execution.

## Main Runtime Path

If you only want to understand the core system, focus on these files first:

1. `Interactive_Demo.ipynb`
2. `sagittarius_env.py`
3. `REAL_ROBOT_TEST_FLOW.md`

Together, they define:

- How natural-language commands are interpreted.
- Which environment APIs the generated code can call.
- How perception and action are grounded in the Sagittarius robot backend.

## In Short

This directory is the control layer above `sagittarius_ws/`.

It does not implement low-level robot drivers itself. Instead, it provides the notebook prompts, execution logic, perception wrapper, and test utilities that allow a language model to generate and run task-level robot policies on the Sagittarius platform.
