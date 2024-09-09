It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
```shell
rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y
```

```shell
sudo apt install python3-colcon-common-extensions
```



# Digital Twin

## Create URDF Model
```shell
sudo apt install ros-${ROS_DISTRO}-urdf-tutorial
```

```shell
ros2 launch urdf_tutorial display.launch.py model:=/media/loidinh/data/WORKSPACE/arm-robot/src/armbot_description/urdf/armbot.urdf.xacro
```

## Visualize the Robot
```shell
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /media/deletal/data/WORKSPACE/arm-robot/src/armbot_description/urdf/armbot.urdf.xacro)"
```

```shell
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

```shell
ros2 run rviz2 rviz2
```

- Change to "world" frame
- Add TF
- Add RobotModel
- Change Robot Description
- Save config

## Launch Files
```shell
ros2 launch armbot_description display.launch.py
```

## Simulate the Robot
```shell
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
sudo apt-get install ros-${ROS_DISTRO}-gazebo-ros-pkgs
```

```shell
ros2 launch armbot_description gazebo.launch.py
```
```shell
pkill -f -9 gzserver
pkill -f -9 gzclient
```

# Control
```shell
sudo apt-get install ros-${ROS_DISTRO}-gazebo-ros2-control
sudo apt-get install ros-${ROS_DISTRO}-ros2-control
```

```shell
ros2 launch armbot_controller controller.launch.py
```

```shell
ros2 launch armbot_controller slider_controller.launch.py
```