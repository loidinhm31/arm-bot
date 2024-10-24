mod serialize;
mod entry;
mod error;

use serialize::joint_state::{
    SerializableJointState,
    SerializableRosString
};
use entry::publish_entry::PublisherEntry;
use futures_util::{SinkExt, StreamExt};
use rclrs;
use rosidl_runtime_rs::Message;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};
use std::any::{Any, TypeId};
use tokio::{net::TcpListener, sync::mpsc};
use tokio_tungstenite::{accept_async, tungstenite::protocol::Message as WsMessage};
use tracing::{debug, error, info, warn};
use uuid::Uuid;

use std_msgs::msg::String as RosString;
use sensor_msgs::msg::JointState;
use crate::error::server_error::{
    ServerError, Result
};

#[derive(Serialize, Deserialize, Debug)]
struct AuthMessage {
    token: String,
}

#[derive(Serialize, Deserialize, Debug)]
struct ClientMessage {
    topic: String,
    msg_type: String,
    msg: Value,
}

#[derive(Clone, Debug)]
struct Client {
    id: Uuid,
    sender: mpsc::UnboundedSender<WsMessage>,
}

struct ServerState {
    clients: HashMap<Uuid, Client>,
    publishers: HashMap<String, PublisherEntry>,
}

impl ServerState {
    fn new() -> Self {
        ServerState {
            clients: HashMap::new(),
            publishers: HashMap::new(),
        }
    }

    fn add_client(&mut self, client: Client) {
        let id = client.id;  // Store id before move
        self.clients.insert(id, client);
        info!("Client {} connected", id);  // Use stored id
    }

    fn remove_client(&mut self, id: Uuid) {
        if self.clients.remove(&id).is_some() {
            info!("Client {} disconnected", id);
        }
    }
}

pub struct RosWebSocketServer {
    node: Arc<rclrs::Node>,
    state: Arc<Mutex<ServerState>>,
    auth_token: String,
}

impl RosWebSocketServer {
    pub fn new(context: &rclrs::Context, node_name: &str, auth_token: String) -> Result<Self> {
        info!("Creating new ROS2 WebSocket server with node name: {}", node_name);
        let node = rclrs::create_node(context, node_name)
            .map_err(|e| {
                error!("Failed to create ROS2 node: {}", e);
                ServerError::Ros(e.to_string())
            })?;

        // Test publisher creation
        let test_pub = node.create_publisher::<RosString>("/test_topic", rclrs::QOS_PROFILE_DEFAULT)
            .map_err(|e| {
                error!("Failed to create test publisher: {}", e);
                ServerError::Ros(e.to_string())
            })?;

        // Try publishing a test message
        let test_msg = RosString { data: "Test message".to_string() };
        if let Err(e) = test_pub.publish(test_msg) {
            error!("Failed to publish test message: {}", e);
        } else {
            info!("Successfully published test message to /test_topic");
        }

        Ok(RosWebSocketServer {
            node,
            state: Arc::new(Mutex::new(ServerState::new())),
            auth_token,
        })
    }
    pub async fn run(&self, addr: &str) -> Result<()> {
        let listener = TcpListener::bind(addr).await.unwrap();
        info!("WebSocket server listening on {}", addr);

        while let Ok((stream, addr)) = listener.accept().await {
            info!("New connection from {}", addr);
            let server_state = self.state.clone();
            let node = self.node.clone();
            let auth_token = self.auth_token.clone();

            tokio::spawn(async move {
                if let Err(e) = Self::handle_connection(stream, server_state, node, auth_token).await {
                    error!("Connection error: {}", e);
                }
            });
        }

        Ok(())
    }

    async fn handle_connection(
        stream: tokio::net::TcpStream,
        state: Arc<Mutex<ServerState>>,
        node: Arc<rclrs::Node>,
        auth_token: String,
    ) -> Result<()> {
        let ws_stream = accept_async(stream).await?;
        let (mut ws_sender, mut ws_receiver) = ws_stream.split();
        let (tx, mut rx) = mpsc::unbounded_channel();
        let client_id = Uuid::new_v4();
        let client = Client {
            id: client_id,
            sender: tx,
        };

        // Authentication
        if let Some(Ok(msg)) = ws_receiver.next().await {
            if let WsMessage::Text(auth_msg) = msg {
                let auth: AuthMessage = serde_json::from_str(&auth_msg)?;
                if auth.token != auth_token {
                    warn!("Authentication failed for client {}", client_id);
                    return Err(ServerError::AuthenticationFailed);
                }
            }
        }

        {
            let mut state = state.lock().unwrap();
            state.add_client(client);  // Removed clone here since we don't need it anymore
        }

        // Handle incoming messages
        let state_clone = state.clone();
        let message_handler = tokio::spawn(async move {
            while let Some(Ok(msg)) = ws_receiver.next().await {
                if let WsMessage::Text(text) = msg {
                    if let Err(e) = Self::handle_message(&text, &node, &state_clone).await {
                        error!("Error handling message: {}", e);
                    }
                }
            }
        });

        // Handle outgoing messages
        let mut send_handler = tokio::spawn(async move {
            while let Some(msg) = rx.recv().await {
                if let Err(e) = ws_sender.send(msg).await {
                    error!("Error sending message: {}", e);
                    break;
                }
            }
        });

        tokio::select! {
            _ = (&mut send_handler) => {},
            _ = message_handler => {},
        }

        {
            let mut state = state.lock().unwrap();
            state.remove_client(client_id);
        }

        Ok(())
    }

    async fn handle_message(
        message: &str,
        node: &rclrs::Node,
        state: &Arc<Mutex<ServerState>>,
    ) -> Result<()> {
        debug!("Received raw message: {}", message);

        let client_msg: ClientMessage = serde_json::from_str(message)?;
        info!("Parsed message - Topic: {}, Type: {}", client_msg.topic, client_msg.msg_type);
        debug!("Message content: {:?}", client_msg.msg);

        let mut state = state.lock().unwrap();

        match client_msg.msg_type.as_str() {
            "sensor_msgs/msg/JointState" | "sensor_msgs/JointState" => {
                info!("Processing JointState message for topic: {}", client_msg.topic);
                let ros_msg: SerializableJointState = match serde_json::from_value(client_msg.msg.clone()) {
                    Ok(msg) => {
                        msg
                    },
                    Err(e) => {
                        error!("Failed to parse JointState message: {}", e);
                        error!("Raw message content: {:?}", client_msg.msg);
                        return Err(ServerError::Json(e));
                    }
                };

                info!("Converting to ROS message...");
                let ros_msg = JointState::from(ros_msg);
                debug!("ROS message content: {:?}", ros_msg);

                Self::publish_message(node, &mut state, &client_msg.topic, ros_msg)?;
            }
            "std_msgs/msg/String" | "std_msgs/String" => {
                info!("Processing String message for topic: {}", client_msg.topic);

                // First, let's try to handle it as a string directly
                let ros_msg = if let Some(str_val) = client_msg.msg.as_str() {
                    SerializableRosString {
                        data: str_val.to_string(),
                    }
                } else {
                    // If it's not a direct string, try to parse as an object
                    match serde_json::from_value::<SerializableRosString>(client_msg.msg.clone()) {
                        Ok(msg) => msg,
                        Err(e) => {
                            error!("Failed to parse String message: {}", e);
                            error!("Attempting alternative parsing...");

                            // If the message is nested differently, try to extract data field
                            if let Some(data) = client_msg.msg.get("data") {
                                if let Some(str_val) = data.as_str() {
                                    SerializableRosString {
                                        data: str_val.to_string(),
                                    }
                                } else {
                                    error!("Data field is not a string: {:?}", data);
                                    return Err(ServerError::Json(e));
                                }
                            } else {
                                error!("No 'data' field found in message");
                                return Err(ServerError::Json(e));
                            }
                        }
                    }
                };

                info!("Converting to ROS message...");
                let ros_msg = RosString::from(ros_msg);
                debug!("Final ROS message content: {:?}", ros_msg);

                Self::publish_message(node, &mut state, &client_msg.topic, ros_msg)?;
            }
            unsupported => {
                error!("Unsupported message type: {}", unsupported);
                return Err(ServerError::UnsupportedMessageType(unsupported.to_string()));
            }
        }

        Ok(())
    }

    fn publish_message<T>(
        node: &rclrs::Node,
        state: &mut ServerState,
        topic: &str,
        message: T,
    ) -> Result<()>
    where
        T: Message + 'static + std::fmt::Debug,
    {
        info!("Publishing message to topic: {}", topic);
        debug!("Message content: {:?}", message);

        let publisher = if let Some(entry) = state.publishers.get(topic) {
            if entry.type_id != TypeId::of::<Arc<rclrs::Publisher<T>>>() {
                info!("Removing publisher with mismatched type for topic: {}", topic);
                state.publishers.remove(topic);
                None
            } else {
                Some(entry)
            }
        } else {
            None
        };

        let entry = if publisher.is_none() {
            info!("Creating new publisher for topic: {}", topic);
            let new_publisher = node
                .create_publisher::<T>(topic, rclrs::QOS_PROFILE_DEFAULT)
                .map_err(|e| {
                    error!("Failed to create publisher for topic {}: {}", topic, e);
                    ServerError::Ros(e.to_string())
                })?;

            info!("Successfully created publisher for topic: {}", topic);
            let entry = PublisherEntry::new(new_publisher);
            state.publishers.insert(topic.to_string(), entry);
            state.publishers.get(topic).unwrap()
        } else {
            publisher.unwrap()
        };

        match entry.publisher.downcast_ref::<Arc<rclrs::Publisher<T>>>() {
            Some(publisher) => {
                debug!("Successfully got publisher reference");
                publisher.publish(message).map_err(|e| {
                    error!("Failed to publish message to topic {}: {}", topic, e);
                    ServerError::Ros(e.to_string())
                })?;
                info!("Successfully published message to topic: {}", topic);
                Ok(())
            }
            None => {
                error!("Failed to downcast publisher for topic: {}", topic);
                Err(ServerError::Ros(format!(
                    "Publisher type mismatch for topic: {}",
                    topic
                )))
            }
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    // Initialize ROS2
    let context = rclrs::Context::new(std::env::args())
        .map_err(|e| ServerError::Ros(e.to_string()))?;

    let server = RosWebSocketServer::new(
        &context,
        "ros2_websocket_node",
        std::env::var("WS_AUTH_TOKEN").unwrap_or_else(|_| "your_secret_token".to_string()),
    )?;

    server.run("0.0.0.0:9090").await
}