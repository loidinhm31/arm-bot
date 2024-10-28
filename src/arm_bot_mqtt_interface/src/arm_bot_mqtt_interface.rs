use rclrs::{Node, Publisher, RclrsError};
use rosidl_runtime_rs::{
    Sequence,
    String as RosString,
};
use rumqttc::v5::{Client, Connection, ConnectionError, Event, Incoming, MqttOptions};
use std::{
    env,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};
use sensor_msgs::msg::rmw::JointState;

struct ArmbotMqttNode {
    node: Arc<Node>,
    mqtt_client: Arc<Client>,
    joint_commands: Arc<Mutex<Vec<f64>>>,
    _command_subscriber: Arc<rclrs::Subscription<JointState>>,
}

impl ArmbotMqttNode {
    fn new(context: &rclrs::Context) -> Result<Arc<Self>, RclrsError> {
        let node = rclrs::create_node(context, "armbot_mqtt_node")?;

        // MQTT setup
        let (mqtt_client, event_loop) = Self::create_mqtt_client("armbot_client")?;
        let mqtt_client = Arc::new(mqtt_client);

        let joint_commands = Arc::new(Mutex::new(vec![0.0; 4]));

        let commands_clone = joint_commands.clone();
        let _command_subscriber = node.create_subscription::<JointState, _>(
            "/joint_states",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: JointState| {
                println!("Received joint states: {:?}", msg.position);
                let mut commands = commands_clone.lock().unwrap();
                *commands = msg.position.to_vec();
            },
        )?;

        let node = Arc::new(ArmbotMqttNode {
            node,
            mqtt_client,
            joint_commands,
            _command_subscriber,
        });

        // Start MQTT event loop
        node.start_mqtt_event_loop(event_loop);

        Ok(node)
    }


    fn create_mqtt_client(client_id: &str) -> Result<(Client, Connection), RclrsError> {
        let mqtt_username = env!("MQTT_USERNAME");
        let mqtt_password = env!("MQTT_PASSWORD");
        let mqtt_server_name = env!("MQTT_SERVER_NAME");
        let mqtt_server_port: u16 = env!("MQTT_SERVER_PORT").parse().unwrap();

        let mut mqtt_options = MqttOptions::new(client_id, mqtt_server_name, mqtt_server_port);
        mqtt_options.set_credentials(mqtt_username, mqtt_password);

        Ok(Client::new(mqtt_options, 10))
    }

    fn start_mqtt_event_loop(self: &Arc<Self>, mut event_loop: Connection) {
        thread::spawn(move || {
            loop {
                for (i, notification) in event_loop.iter().enumerate() {
                    match notification {
                        Ok(Event::Incoming(Incoming::ConnAck(_))) => {
                            println!("Connected to MQTT broker");
                        }
                        Err(ConnectionError::Io(error)) if error.kind() == std::io::ErrorKind::ConnectionRefused => {
                            println!("Failed to connect to the server. Make sure correct client is configured properly!\nError: {error:?}");
                            return;
                        }
                        _ => {
                            println!("{i}. Notification = {notification:?}");
                        }
                    }
                }
            }
        });
    }

    fn run(self: &Arc<Self>) -> Result<(), RclrsError> {
        self.run_publisher_mqtt()?;
        Ok(())
    }

    fn run_publisher_mqtt(&self) -> Result<(), RclrsError> {
        let mqtt_client = self.mqtt_client.clone();
        let joint_commands = self.joint_commands.clone();

        thread::spawn(move || {
            loop {
                thread::sleep(Duration::from_millis(1500));
                let commands = joint_commands.lock().unwrap().clone();
                let msg = Self::format_joint_command(&commands);
                println!("Sending new command: {}", msg);
                if let Err(e) = mqtt_client.publish("armbot/commands", rumqttc::v5::mqttbytes::QoS::AtLeastOnce, false, msg.into_bytes()) {
                    eprintln!("Failed to send MQTT command: {:?}", e);
                }
            }
        });

        Ok(())
    }

    fn format_joint_command(commands: &[f64]) -> String {
        let base = ((commands[0] + std::f64::consts::PI / 2.0) * 180.0 / std::f64::consts::PI) as i32;
        let shoulder = 180 - ((commands[1] + std::f64::consts::PI / 2.0) * 180.0 / std::f64::consts::PI) as i32;
        let elbow = ((commands[2] + std::f64::consts::PI / 2.0) * 180.0 / std::f64::consts::PI) as i32;
        let gripper = ((-commands[3]) * 180.0 / (std::f64::consts::PI / 2.0)) as i32;

        format!(
            "{}{}{}{}",
            Self::format_single_joint('b', base),
            Self::format_single_joint('s', shoulder),
            Self::format_single_joint('e', elbow),
            Self::format_single_joint('g', gripper)
        )
    }

    fn format_single_joint(joint: char, angle: i32) -> String {
        format!("{}{}{},", joint, Self::compensate_zeros(angle), angle)
    }

    fn compensate_zeros(value: i32) -> String {
        if value < 10 {
            "00".to_string()
        } else if value < 100 {
            "0".to_string()
        } else {
            "".to_string()
        }
    }

    fn parse_feedback(feedback: &str) -> Result<Vec<f64>, std::num::ParseFloatError> {
        println!("Parsing feedback: {}", feedback);
        let mut joint_states = vec![0.0; 4];
        for part in feedback.split(',') {
            if part.len() < 2 {
                continue;
            }
            let joint = part.chars().next().unwrap();
            let angle: f64 = part[1..].parse()?;
            let radians = match joint {
                'b' => (angle - 180.0) * std::f64::consts::PI / 180.0,
                's' => (180.0 - angle) * std::f64::consts::PI / 180.0,
                'e' => (angle - 90.0) * std::f64::consts::PI / 180.0,
                'g' => -angle * std::f64::consts::PI / 360.0,
                _ => continue,
            };
            let index = match joint {
                'b' => 0,
                's' => 1,
                'e' => 2,
                'g' => 3,
                _ => continue,
            };
            joint_states[index] = radians;
        }
        Ok(joint_states)
    }

    fn update_and_publish_joint_states(
        joint_state_publisher: &Arc<Publisher<JointState>>,
        new_joint_states: Vec<f64>,
    ) -> Result<(), RclrsError> {
        println!("Publishing joints {:?}", new_joint_states.clone());

        let duration_since_epoch = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap();

        let header = std_msgs::msg::rmw::Header {
            stamp: builtin_interfaces::msg::rmw::Time {
                sec: duration_since_epoch.as_secs() as i32,
                nanosec: duration_since_epoch.subsec_nanos(),
            },
            frame_id: RosString::from("joint"),
        };

        let mut joint_state_msg = JointState::default();
        joint_state_msg.header = header;
        joint_state_msg.name = Sequence::from_iter(vec![
            "base_joint", "shoulder_joint", "elbow_joint", "gripper_joint"
        ].into_iter().map(RosString::from));
        let mut position_sequence = Sequence::new(new_joint_states.len());
        for (i, &position) in new_joint_states.iter().enumerate() {
            position_sequence[i] = position;
        }
        joint_state_msg.position = position_sequence;

        joint_state_publisher.publish(&joint_state_msg)
            .expect("failed to publish message");
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    let context = rclrs::Context::new(env::args())?;

    let node = ArmbotMqttNode::new(&context)?;
    node.run()?;

    let executor = rclrs::SingleThreadedExecutor::new();
    executor.add_node(&node.node)?;
    executor.spin().map_err(|err| err.into())
}