use rclrs::RclrsError;
use rumqttc::v5::mqttbytes::QoS;
use rumqttc::v5::{Client, ConnectionError, MqttOptions};
use sensor_msgs::msg::JointState;
use std::f64::consts::PI;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

struct ArmbotMqttInterface {
    node: Arc<rclrs::Node>,
    mqtt_client: Arc<Mutex<Client>>,
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
        let (mqtt_client, mut eventloop) = Client::new(mqtt_options, 10);
        let mqtt_client = Arc::new(Mutex::new(mqtt_client));

        let joint_commands = Arc::new(Mutex::new(vec![0.0; 4]));
        let joint_states = Arc::new(Mutex::new(vec![0.0; 4]));
        let new_command_received = Arc::new(Mutex::new(false));

        let commands_clone = joint_commands.clone();
        let new_command_received_clone = new_command_received.clone();
        let _command_subscriber = node.create_subscription::<JointState, _>(
            "/joint_commands",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: JointState| {
                println!("Received joint command: {:?}", msg.position);
                let mut commands = commands_clone.lock().unwrap();
                *commands = msg.position;
                *new_command_received_clone.lock().unwrap() = true;
            },
        )?;

        println!("Subscription setup complete");

        // Start MQTT event loop in a separate thread
        thread::spawn(move || {
            for (i, notification) in eventloop.iter().enumerate() {
                match notification {
                    Err(ConnectionError::Io(error))
                    if error.kind() == std::io::ErrorKind::ConnectionRefused =>
                        {
                            println!("Failed to connect to the server. Make sure correct client is configured properly!\nError: {error:?}");
                            return;
                        }
                    _ => {}
                }
                println!("{i}. Notification = {notification:?}");
            }
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

    fn run(&self) -> Result<(), RclrsError> {
        let mqtt_client = self.mqtt_client.clone();
        let joint_commands = self.joint_commands.clone();
        let joint_states = self.joint_states.clone();
        let new_command_received = self.new_command_received.clone();

        // Spawn a thread for MQTT publishing
        thread::spawn(move || {
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

                    let mut msg = String::new();
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
                    if let Err(e) = mqtt_client.lock().unwrap().publish("armbot/commands", QoS::AtLeastOnce, false, msg.into_bytes()) {
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
    let _result = interface.run();
    interface.on_deactivate()?;

    Ok(())
}