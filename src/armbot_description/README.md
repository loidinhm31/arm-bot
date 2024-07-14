# Create URDF Model
```shell
sudo apt install ros-humble-urdf-tutorial
```

```shell
ros2 launch urdf_tutorial display.launch.py model:=/media/deletal/data/WORKSPACE/arm-robot/src/armbot_description/urdf/arduinobot.urdf.xacro
```

# Visualize robot
```shell
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /media/deletal/data/WORKSPACE/arm-robot/src/armbot_description/urdf/arduinobot.urdf.xacro)"
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

# Launch File
```shell
ros2 launch armbot_description display.launch.py
```