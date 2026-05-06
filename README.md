# AIR 5051 Final Project - Team 7

## Project Title
LLM-Based Embodied Control with Code Generation and Self-Correction

## Project Overview
This repository contains the final submission materials for Team 7's AIR 5051 final project. The project integrates a language-based policy generation workflow with the Sagittarius robotic arm platform, enabling natural-language task instructions to be translated into executable robot actions.

At a high level, the system connects:

- a notebook-based language interaction layer,
- a Python environment wrapper for perception and action execution, and
- a ROS workspace for Sagittarius robot control, motion planning, and device integration.

## Repository Structure
- `src/code_as_policies/`: notebook workflow, language-policy runtime, environment wrapper, and helper scripts
- `src/sagittarius_ws/`: ROS workspace source packages for Sagittarius control, perception, MoveIt, and demos
- `docs/`: final report and supplementary materials

## Main Components
- `src/code_as_policies/Interactive_Demo.ipynb`
  Main notebook entry point for running the language-to-robot interaction workflow.
- `src/code_as_policies/sagittarius_env.py`
  Python bridge between the notebook layer and the ROS backend, including perception updates, world-state handling, and action execution.
- `src/code_as_policies/scripts/`
  Utility scripts for backend checking, smoke testing, and action cancellation.
- `src/sagittarius_ws/sagittarius_arm_ros/`
  ROS packages used for robot description, motion planning, perception, SDK integration, and demos.

## Environment and Dependencies
- Ubuntu with ROS Noetic recommended
- Python 3
- Jupyter Notebook
- Conda environment file: `src/code_as_policies/environment.interactive_demo.yml`

## Real Robot Test Flow
The original operator workflow is documented in `src/code_as_policies/REAL_ROBOT_TEST_FLOW.md`. This section mirrors that process and adapts the paths to this submission repository.

Assume:

- catkin workspace: `~/LMP/sagittarius_ws`

Before running the system, copy the ROS source tree from this repository into the catkin workspace:

```bash
mkdir -p ~/LMP/sagittarius_ws/src
cp -R src/sagittarius_ws/* ~/LMP/sagittarius_ws/src/
```

### 0. Device check
```bash
ls -l /dev/sagittarius
ls -l /dev/usb_cam
```

If `/dev/usb_cam` does not exist, replace `video_dev:=/dev/usb_cam` with `video_dev:=/dev/video0` in the camera launch command below.

### 1. Build
```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 2. Terminal A: launch MoveIt and the real robot driver
```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sagittarius_moveit demo_true.launch robot_name:=sgr532
```

### 3. Terminal B: launch `SGRCtrl`
```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun sagittarius_object_color_detector sgr_ctrl.py __ns:=/sgr532 _robot_name:=sgr532
```

### 4. Terminal C: launch the camera
```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sagittarius_object_color_detector usb_cam.launch video_dev:=/dev/usb_cam
```

Alternative:

```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sagittarius_object_color_detector usb_cam.launch video_dev:=/dev/video0
```

### 5. Terminal D: check backend connectivity
From the repository root:

```bash
bash src/code_as_policies/scripts/check_ws_backend.sh --arm-name sgr532
```

Manual checks:

```bash
source /opt/ros/noetic/setup.bash
rostopic list | rg '/sgr532/sgr_ctrl|follow_joint_trajectory|sagittarius_joint_states'
rostopic echo -n1 /usb_cam/image_raw/header
rosservice list | rg '/sgr532/get_servo_info|/sgr532/get_robot_info'
```

### 6. Terminal D: smoke test
From the repository root, after the `~/LMP/sagittarius_ws` workspace has been built and sourced:

```bash
python3 src/code_as_policies/scripts/sgr_ctrl_smoke_test.py \
  --arm-name sgr532 \
  --x 0.20 --y 0.00 --z 0.10
```

### 7. Terminal E: launch the notebook
Install the kernel once if needed:

```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m ipykernel install --user --name ros-noetic-py3 --display-name "Python 3 (ROS Noetic)"
```

Start the notebook from the repository root:

```bash
cd src/code_as_policies
jupyter notebook Interactive_Demo.ipynb
```

## Offline Check
From the repository root:

```bash
python3 -m py_compile src/code_as_policies/sagittarius_env.py
```

## Final Report
- `docs/Team7_FinalReport.pdf`

## Notes
- Build artifacts, cache files, and editor metadata are intentionally excluded from this submission repository.
- If Qualcomm-specific code or modules are required by the course submission rules, they should be documented in the report appendix and clearly identified in the repository.
