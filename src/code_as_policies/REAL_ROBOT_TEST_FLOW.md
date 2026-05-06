# Sagittarius 真机测试速查表

仓库根目录假设为：

```bash
cd ~/LMP
```

## 0. 设备检查

```bash
ls -l /dev/sagittarius
ls -l /dev/usb_cam
```

没有 `/dev/usb_cam` 就把后面的 `video_dev:=/dev/usb_cam` 改成 `video_dev:=/dev/video0`。

## 1. 编译

```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 2. 终端 A：启动 MoveIt + 真机驱动

```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sagittarius_moveit demo_true.launch robot_name:=sgr532
#roslaunch +path
```

## 3. 终端 B：启动 `SGRCtrl`

```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun sagittarius_object_color_detector sgr_ctrl.py __ns:=/sgr532 _robot_name:=sgr532
#rosrun +path
#choose path(2)
```

## 4. 终端 C：启动相机

```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sagittarius_object_color_detector usb_cam.launch video_dev:=/dev/usb_cam
#roslaunch +path
```

或：

```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sagittarius_object_color_detector usb_cam.launch video_dev:=/dev/video0
```

## 5. 终端 D：检查联通

```bash
cd ~/LMP
bash code_as_policies/scripts/check_ws_backend.sh --arm-name sgr532
```

手动检查：

```bash
source /opt/ros/noetic/setup.bash
rostopic list | rg '/sgr532/sgr_ctrl|follow_joint_trajectory|sagittarius_joint_states'
rostopic echo -n1 /usb_cam/image_raw/header
rosservice list | rg '/sgr532/get_servo_info|/sgr532/get_robot_info'
```

## 6. 终端 D：小动作烟雾测试

```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 ../code_as_policies/scripts/sgr_ctrl_smoke_test.py \
  --arm-name sgr532 \
  --x 0.20 --y 0.00 --z 0.10
```

## 7. 终端 E：启动 notebook

首次可先装 kernel：

```bash
cd ~/LMP/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m ipykernel install --user --name ros-noetic-py3 --display-name "Python 3 (ROS Noetic)"
```

启动：

```bash
cd ~/LMP/code_as_policies
jupyter notebook Interactive_Demo.ipynb
```

## 8. Notebook 内执行

初始化：

```python
from sagittarius_env import SagittariusEnv
env = SagittariusEnv(arm_name='sgr532')
```

刷新：

```python
env.refresh_world_state(force=True)
env.object_list
```

按 notebook 原流程初始化到可执行：

```python
put_first_on_second(...)
```

或：

```python
lmp_tabletop_ui(...)
```

## 9. 推荐测试命令

直接抓放：

```python
put_first_on_second('blue block', 'green bowl')
put_first_on_second('red block', 'area a')
```

自然语言入口：

```python
lmp_tabletop_ui("put blue block into green bowl", f'objects = {env.object_list}')
```

## 10. 本次改动重点看什么

- 抓空后是否回观察位
- `world_state` 是否刷新
- 第二次 pick 是否用新坐标
- 目标不可见时是否不再 place
- notebook 失败时是否只打印原因、不直接中断

## 11. 取消动作

```bash
cd ~/LMP
bash code_as_policies/scripts/cancel_sgr_goal.sh --arm-name sgr532
```

或：

```bash
rostopic pub -1 /sgr532/sgr_ctrl/cancel actionlib_msgs/GoalID '{}'
```

## 12. 离线检查

```bash
cd ~/LMP
python3 -m py_compile code_as_policies/sagittarius_env.py
```
