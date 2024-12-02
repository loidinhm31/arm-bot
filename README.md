# Build
```shell
vcs import src < src/ros2_rust/ros2_rust_humble.repos
```

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


# Kinematics

## TF2 Transform
```shell
sudo apt-get install ros-${ROS_DISTRO}-tf-transformations
```

```shell
sudo pip3 install transforms3d
```

## MoveIt2
```shell
sudo apt-get install ros-${ROS_DISTRO}-moveit-planners*
sudo apt-get install ros-${ROS_DISTRO}-moveit-ros-visualization
sudo apt-get install ros-${ROS_DISTRO}-moveit*
sudo apt-get install ros-${ROS_DISTRO}-moveit-planners-ompl && sudo apt-get install ros-${ROS_DISTRO}-ompl
```

Go into the "Context" menu and then change the planner to "OMPL"

## Alexa
https://developer.amazon.com/en-US/docs/alexa/alexa-skills-kit-sdk-for-python/set-up-the-sdk.html
```shell
pip install flask
pip install ask-sdk
pip install flask-ask-sdk
```

```shell
ros2 launch arm_bot_bringup simulated_robot.launch.py
```

## Rosbridge Websocket SSL connection
### OpenSSL
Key:
```shell
openssl genrsa -out server_key.pem 2048
```

Certificate Signing Request:
```shell
openssl req -new -key server_key.pem -out server_csr.pem
```

Certificate:
```shell
openssl x509 -req -days 1825 -in server_csr.pem -signkey server_key.pem -out server_cert.pem
```

### Solution for working with self-signed certificates
Open the URL:PORT of secure websocket-server in the browser

https://127.0.0.1:9090

Or from a remote machine:
https://10.3.10.199:9090

A security warning will appear, asking you to confirm or decline the self-signed certificate.

------------------------------------------------------
colcon build --packages-select armbot_mqtt_interface

ros2 launch armbot_bringup arm_bot.launch.py

*ros2 launch armbot_mqtt_interface armbot_mqtt_interface


ros2 run armbot_mqtt_interface armbot_mqtt_interface

ros2 topic pub /joint_states sensor_msgs/msg/JointState '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, name: ['base_joint', 'shoulder_joint', 'elbow_joint', 'gripper_joint'], position: [2.0, 5.0, 15.0, 0.0], velocity: [], effort: []}'

mosquitto_sub -h 127.0.0.1 -t "armbot/commands" -u "armbot" -P "18s=799G"