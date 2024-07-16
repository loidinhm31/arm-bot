# Digital Twin

## Create URDF Model
```shell
sudo apt install ros-humble-urdf-tutorial
```

```shell
ros2 launch urdf_tutorial display.launch.py model:=/media/deletal/data/WORKSPACE/arm-robot/src/armbot_description/urdf/arm.urdf.xacro
```

## Visualize the Robot
```shell
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /media/deletal/data/WORKSPACE/arm-robot/src/armbot_description/urdf/arm.urdf.xacro)"
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
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

```shell
ros2 launch armbot_description gazebo.launch.py
```