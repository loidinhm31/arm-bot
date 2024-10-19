use builtin_interfaces::msg::rmw::Time;
use rclrs::{Publisher, RclrsError};
use rosidl_runtime_rs::{Sequence, String as RosString};
use rumqttc::v5::mqttbytes::QoS;
use rumqttc::v5::{Client, Connection, ConnectionError, Event, Incoming, MqttOptions};
use sensor_msgs::msg::rmw::JointState;
use std::f64::consts::PI;
use std::num::ParseFloatError;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use std_msgs::msg::rmw::Header;

struct ArmbotMqttInterface {
    node: Arc<rclrs::Node>,
    mqtt_client: Client,
    joint_commands: Arc<Mutex<Vec<f64>>>,
    joint_states: Arc<Mutex<Vec<f64>>>,
    new_command_received: Arc<Mutex<bool>>,
    _command_subscriber: Arc<rclrs::Subscription<JointState>>,
}

impl ArmbotMqttInterface {
    fn new() -> Result<Self, RclrsError> {
        let mqtt_username = env!("MQTT_USERNAME");
        let mqtt_password = env!("MQTT_PASSWORD");
        let mqtt_server_name = env!("MQTT_SERVER_NAME");
        let mqtt_server_port: u16 = env!("MQTT_SERVER_PORT").parse().unwrap();

        let ctx = rclrs::Context::new(std::env::args())?;
        let node = rclrs::create_node(&ctx, "armbot_mqtt_interface")?;

        // MQTT setup
        let mut mqtt_options = MqttOptions::new("armbot_client", mqtt_server_name, mqtt_server_port);
        mqtt_options.set_credentials(mqtt_username, mqtt_password);
        mqtt_options.set_keep_alive(Duration::from_secs(5));
        let (mqtt_client, eventloop) = Client::new(mqtt_options, 10);

        let joint_commands = Arc::new(Mutex::new(vec![0.0; 4]));
        let joint_states = Arc::new(Mutex::new(vec![0.0; 4]));
        let new_command_received = Arc::new(Mutex::new(false));

        let commands_clone = joint_commands.clone();
        let new_command_received_clone = new_command_received.clone();
        let _command_subscriber = node.create_subscription::<JointState, _>(
            "/joint_commands",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: sensor_msgs::msg::rmw::JointState| {
                println!("Received joint command: {:?}", msg.position);
                let mut commands = commands_clone.lock().unwrap();
                *commands = msg.position.to_vec();
                *new_command_received_clone.lock().unwrap() = true;
            },
        )?;

        let joint_state_publisher = node.create_publisher::<sensor_msgs::msg::rmw::JointState>("/joint_states", rclrs::QOS_PROFILE_DEFAULT)?;
        // Start MQTT event loop in a separate thread
        let joint_states_clone = joint_states.clone();
        let joint_state_publisher_clone = joint_state_publisher.clone();
        thread::spawn(move || {
            Self::mqtt_event_loop(eventloop, joint_states_clone, joint_state_publisher_clone);
        });

        Ok(ArmbotMqttInterface {
            node,
            mqtt_client,
            joint_commands,
            joint_states,
            new_command_received,
            _command_subscriber,
        })
    }

    fn on_activate(&mut self) -> Result<(), RclrsError> {
        println!("Starting robot hardware ...");
        *self.joint_commands.lock().unwrap() = vec![0.0; 4];
        *self.joint_states.lock().unwrap() = vec![0.0; 4];
        println!("MQTT Interface started, ready to take commands");
        Ok(())
    }

    fn on_deactivate(&mut self) -> Result<(), RclrsError> {
        println!("Stopping robot MQTT Interface ...");
        println!("MQTT Interface stopped");
        Ok(())
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

    fn format_joint_command(joint: char, angle: i32) -> String {
        format!("{}{}{},", joint, Self::compensate_zeros(angle), angle)
    }

    fn update_and_publish_joint_states(
        joint_states: &Arc<Mutex<Vec<f64>>>,
        joint_state_publisher: &Arc<Publisher<JointState>>,
        new_joint_states: Vec<f64>,
    ) -> Result<(), RclrsError> {
        // Update joint states
        {
            println!("Publishing joints {:?}", new_joint_states.clone());
            let mut states = joint_states.lock().map_err(|e| format!("Failed to lock joint states: {:?}", e)).unwrap();
            *states = new_joint_states.clone();
        }

        // Create and publish JointState message
        let mut joint_state_msg = JointState {
            header: Header {
                stamp: Time::default(),
                frame_id: RosString::from(""),
            },
            name: Sequence::from_iter(vec![
                "base_joint", "shoulder_joint", "elbow_joint", "gripper_joint"
            ].into_iter().map(RosString::from)),
            position: Sequence::from_iter(new_joint_states.iter().cloned()),
            velocity: Sequence::new(0),
            effort: Sequence::new(0),
        };

        println!("Joint_state message: {:?}", joint_state_msg);

        joint_state_publisher.publish(&joint_state_msg)?;
        Ok(())
    }

    fn parse_feedback(feedback: &str) -> Result<Vec<f64>, ParseFloatError> {
        println!("Parsing feedback: {}", feedback);
        let mut joint_states = vec![0.0; 4];
        for part in feedback.split(',') {
            if part.len() < 2 {
                continue;
            }
            let joint = part.chars().next().unwrap();
            let angle: f64 = part[1..].parse()?;
            let radians = match joint {
                'b' => (angle - 180.0) * PI / 180.0,
                's' => (180.0 - angle) * PI / 180.0,
                'e' => (angle - 90.0) * PI / 180.0,
                'g' => -angle * PI / 360.0,
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

    fn mqtt_event_loop(
        mut eventloop: Connection,
        joint_states: Arc<Mutex<Vec<f64>>>,
        joint_state_publisher: Arc<Publisher<JointState>>,
    ) {
        for (i, notification) in eventloop.iter().enumerate() {
            match notification {
                Ok(Event::Incoming(Incoming::Publish(publish))) => {
                    let topic = publish.topic;
                    let payload = String::from_utf8_lossy(&publish.payload);
                    println!("Received message on topic: {:?}", topic);
                    println!("Message payload: {}", payload);

                    if topic == "armbot/feedback" {
                        match Self::parse_feedback(&payload) {
                            Ok(new_joint_states) => {
                                if let Err(e) = Self::update_and_publish_joint_states(
                                    &joint_states,
                                    &joint_state_publisher,
                                    new_joint_states,
                                ) {
                                    eprintln!("Error updating and publishing joint states: {:?}", e);
                                }
                            }
                            Err(e) => eprintln!("Error parsing feedback: {:?}", e),
                        }
                    }
                }
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

    fn run(&mut self) -> Result<(), RclrsError> {
        let mqtt_client = self.mqtt_client.clone();
        let joint_commands = self.joint_commands.clone();
        let joint_states = self.joint_states.clone();
        let new_command_received = self.new_command_received.clone();

        // Spawn a thread for MQTT publishing
        thread::spawn(move || {
            mqtt_client.subscribe("armbot/feedback", QoS::AtLeastOnce).unwrap();

            loop {
                thread::sleep(Duration::from_millis(100));

                let should_publish = {
                    let mut new_command = new_command_received.lock().unwrap();
                    if *new_command {
                        *new_command = false;
                        true
                    } else {
                        false
                    }
                };

                if should_publish {
                    let commands = joint_commands.lock().unwrap().clone();
                    *joint_states.lock().unwrap() = commands.clone();

                    let base = ((commands[0] + PI / 2.0) * 180.0 / PI) as i32;
                    let shoulder = 180 - ((commands[1] + PI / 2.0) * 180.0 / PI) as i32;
                    let elbow = ((commands[2] + PI / 2.0) * 180.0 / PI) as i32;
                    let gripper = ((-commands[3]) * 180.0 / (PI / 2.0)) as i32;

                    let msg = format!(
                        "{}{}{}{}",
                        Self::format_joint_command('b', base),
                        Self::format_joint_command('s', shoulder),
                        Self::format_joint_command('e', elbow),
                        Self::format_joint_command('g', gripper)
                    );
                    println!("Sending new command: {}", msg);
                    if let Err(e) = mqtt_client.publish("armbot/commands", QoS::AtLeastOnce, false, msg.into_bytes()) {
                        eprintln!("Failed to send MQTT command: {:?}", e);
                    }
                }
            }
        });

        loop {
            rclrs::spin(self.node.clone())?;
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let mut interface = ArmbotMqttInterface::new()?;

    interface.on_activate()?;
    let result = interface.run();
    interface.on_deactivate()?;

    if let Err(e) = result {
        eprintln!("Error during run: {:?}", e);
    }

    Ok(())
}