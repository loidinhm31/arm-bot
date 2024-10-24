use serde::{Deserialize, Serialize};
use std_msgs::msg::String as RosString;
use sensor_msgs::msg::JointState;

#[derive(Serialize, Deserialize)]
struct SerializableHeader {
    stamp: SerializableTime,
    frame_id: String,
}

#[derive(Serialize, Deserialize)]
struct SerializableTime {
    sec: i32,
    nanosec: u32,
}

impl From<SerializableHeader> for std_msgs::msg::Header {
    fn from(header: SerializableHeader) -> Self {
        std_msgs::msg::Header {
            stamp: builtin_interfaces::msg::Time {
                sec: header.stamp.sec,
                nanosec: header.stamp.nanosec,
            },
            frame_id: header.frame_id,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub(crate) struct SerializableRosString {
    pub(crate) data: String,
}

impl From<SerializableRosString> for RosString {
    fn from(msg: SerializableRosString) -> Self {
        RosString { data: msg.data }
    }
}

#[derive(Serialize, Deserialize)]
pub(crate) struct SerializableJointState {
    header: Option<SerializableHeader>,  // Changed to use SerializableHeader
    name: Vec<String>,
    position: Vec<f64>,
    velocity: Vec<f64>,
    effort: Vec<f64>,
}

impl From<SerializableJointState> for JointState {
    fn from(msg: SerializableJointState) -> Self {
        JointState {
            header: msg.header.map(|h| h.into()).unwrap_or_default(),
            name: msg.name,
            position: msg.position,
            velocity: msg.velocity,
            effort: msg.effort,
        }
    }
}
