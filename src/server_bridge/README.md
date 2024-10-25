```json
{
  "token": "eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJzdWIiOiJkNTM1YjBlOC02NjM2LTRmMTgtYTE0Ni1lNmU4YjAzZDI2YTIiLCJleHAiOjE3Mjk4Njc1MDQsImlhdCI6MTcyOTc4MTEwNH0.yKit579Na_EpsN19RE8axghiRu4ucgOl91cGDKx9gD"
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