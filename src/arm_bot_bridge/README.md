```json
{
  "token": "your_secret_token"
}

```

```json
{
  "topic": "/chatter",
  "msg_type": "std_msgs/msg/String",
  "msg": {
    "data": "Hello ROS2!"
  }
}
```

```json
{
  "topic": "/joint_states",
  "msg_type": "sensor_msgs/msg/JointState",
  "msg": {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "base_link"
    },
    "name": [
      "joint1",
      "joint2",
      "joint3"
    ],
    "position": [
      1.0,
      -0.5,
      0.0
    ],
    "velocity": [
      0.0,
      0.0,
      0.0
    ],
    "effort": [
      0.0,
      0.0,
      0.0
    ]
  }
}
```