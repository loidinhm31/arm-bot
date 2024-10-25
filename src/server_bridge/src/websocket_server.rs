mod def;
mod entry;
mod error;

use def::joint_state::{
    SerializableJointState,
    SerializableRosString,
};

use chrono::{Duration, Utc};
use entry::publish_entry::PublisherEntry;
use error::server_error::{
    Result, ServerError,
};
use futures_util::{SinkExt, StreamExt};
use jsonwebtoken::{decode, encode, DecodingKey, EncodingKey, Header, Validation};
use rclrs;
use rosidl_runtime_rs::Message;
use sensor_msgs::msg::JointState;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::any::TypeId;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};
use std_msgs::msg::String as RosString;
use tokio::{net::TcpListener, sync::mpsc};
use tokio_tungstenite::{accept_async, tungstenite::protocol::Message as WsMessage};
use tracing::{debug, error, info, warn};
use uuid::Uuid;

use def::auth::{
    AuthMessage,
    TokenResponse,
};
use crate::def::auth::{Claims, LoginCredentials, User};

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
        let id = client.id;
        self.clients.insert(id, client);
        info!("Client {} connected", id);
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
    jwt_secret: String,
}

impl RosWebSocketServer {
    pub fn new(context: &rclrs::Context, node_name: &str) -> Result<Self> {
        info!("Creating new ROS2 WebSocket server with node name: {}", node_name);
        let node = rclrs::create_node(context, node_name)
            .map_err(|e| {
                error!("Failed to create ROS2 node: {}", e);
                ServerError::Ros(e.to_string())
            })?;

        // Get JWT secret from environment (set by auth server)
        let jwt_secret = env!("JWT_SECRET");

        Ok(RosWebSocketServer {
            node,
            state: Arc::new(Mutex::new(ServerState::new())),
            jwt_secret: jwt_secret.to_string(),
        })
    }

    pub async fn run(&self, addr: &str) -> Result<()> {
        let listener = TcpListener::bind(addr).await.unwrap();
        info!("WebSocket server listening on {}", addr);

        while let Ok((stream, addr)) = listener.accept().await {
            info!("New connection from {}", addr);
            let server_state = self.state.clone();
            let node = self.node.clone();
            let jwt_secret = self.jwt_secret.clone();

            tokio::spawn(async move {
                if let Err(e) = Self::handle_connection(stream, server_state, node, jwt_secret).await {
                    error!("Connection error: {}", e);
                }
            });
        }

        Ok(())
    }

    fn validate_token(secret: &str, token: &str) -> Result<Claims> {
        let validation = Validation::default();
        let token_data = decode::<Claims>(
            token,
            &DecodingKey::from_secret(secret.as_bytes()),
            &validation,
        ).map_err(|e| ServerError::Auth(format!("Token validation failed: {}", e)))?;

        Ok(token_data.claims)
    }

    pub fn authenticate(&self, credentials: &LoginCredentials) -> Result<TokenResponse> {
        let valid_user = User {
            username: "admin".to_string(),
            password: "password123".to_string(),
        };

        if credentials.username == valid_user.username && credentials.password == valid_user.password {
            // Generate JWT token if credentials are valid
            let expiration = Utc::now()
                .checked_add_signed(Duration::hours(24))
                .expect("valid timestamp")
                .timestamp();

            let claims = Claims {
                sub: credentials.username.clone(),  // Use username as subject
                exp: expiration,
                iat: Utc::now().timestamp(),
            };

            let token = encode(
                &Header::default(),
                &claims,
                &EncodingKey::from_secret(self.jwt_secret.as_bytes()),
            ).map_err(|e| ServerError::Auth(format!("Token generation failed: {}", e)))?;

            Ok(TokenResponse {
                token,
                expires_in: expiration - Utc::now().timestamp(),
            })
        } else {
            Err(ServerError::Auth("Invalid credentials".to_string()))
        }
    }

    async fn handle_connection(
        stream: tokio::net::TcpStream,
        state: Arc<Mutex<ServerState>>,
        node: Arc<rclrs::Node>,
        jwt_secret: String,
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
        let authenticated = match ws_receiver.next().await {
            Some(Ok(WsMessage::Text(auth_msg))) => {
                match serde_json::from_str::<AuthMessage>(&auth_msg) {
                    Ok(auth) => {
                        match Self::validate_token(&jwt_secret, &auth.token) {
                            Ok(claims) => {
                                let current_time = Utc::now().timestamp();
                                if claims.exp < current_time {
                                    error!("Token has expired for client {}", client_id);
                                    if let Err(e) = ws_sender.send(WsMessage::Text(
                                        json!({
                                        "type": "auth",
                                        "authenticated": false,
                                        "error": "Authentication failed: Token expired"
                                    }).to_string()
                                    )).await {
                                        error!("Failed to send error message: {}", e);
                                    }
                                    false
                                } else {
                                    info!("Token validated successfully for client {} (user: {})",
                                      client_id, claims.sub);
                                    // Send authentication success message
                                    if let Err(e) = ws_sender.send(WsMessage::Text(
                                        json!({
                                        "type": "auth",
                                        "authenticated": true,
                                        "clientId": client_id.to_string(),
                                        "username": claims.sub
                                    }).to_string()
                                    )).await {
                                        error!("Failed to send authentication success message: {}", e);
                                    }
                                    true
                                }
                            }
                            Err(e) => {
                                error!("Token validation failed for client {}: {}", client_id, e);
                                if let Err(e) = ws_sender.send(WsMessage::Text(
                                    json!({
                                    "type": "auth",
                                    "authenticated": false,
                                    "error": "Authentication failed: Invalid token"
                                }).to_string()
                                )).await {
                                    error!("Failed to send error message: {}", e);
                                }
                                false
                            }
                        }
                    }
                    Err(e) => {
                        error!("Failed to parse auth message for client {}: {}", client_id, e);
                        if let Err(e) = ws_sender.send(WsMessage::Text(
                            json!({
                            "type": "auth",
                            "authenticated": false,
                            "error": "Authentication failed: Invalid message format"
                        }).to_string()
                        )).await {
                            error!("Failed to send error message: {}", e);
                        }
                        false
                    }
                }
            }
            _ => false,
        };

        if !authenticated {
            warn!("Authentication failed for client {}", client_id);
            return Err(ServerError::AuthenticationFailed);
        }

        info!("Client {} authenticated successfully", client_id);

        {
            let mut state = state.lock().unwrap();
            state.add_client(client);
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

        let client_msg: ClientMessage = serde_json::from_str(message).unwrap();
        info!("Parsed message - Topic: {}, Type: {}", client_msg.topic, client_msg.msg_type);
        debug!("Message content: {:?}", client_msg.msg);

        let mut state = state.lock().unwrap();

        match client_msg.msg_type.as_str() {
            "sensor_msgs/msg/JointState" | "sensor_msgs/JointState" => {
                info!("Processing JointState message for topic: {}", client_msg.topic);
                let ros_msg: SerializableJointState = match serde_json::from_value(client_msg.msg.clone()) {
                    Ok(msg) => {
                        msg
                    }
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
                error!("Unsupported message def: {}", unsupported);
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
                info!("Removing publisher with mismatched def for topic: {}", topic);
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
                    "Publisher def mismatch for topic: {}",
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

    let server = RosWebSocketServer::new(&context, "ros2_websocket_node")?;

    // Start WebSocket server
    info!("Starting WebSocket server on ws://0.0.0.0:9090");
    server.run("0.0.0.0:9090").await?;

    Ok(())
}