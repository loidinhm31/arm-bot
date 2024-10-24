use thiserror::Error;

#[derive(Error, Debug)]
pub enum ServerError {
    #[error("WebSocket error: {0}")]
    WebSocket(#[from] tokio_tungstenite::tungstenite::Error),
    #[error("JSON serialization error: {0}")]
    Json(#[from] serde_json::Error),
    #[error("ROS2 error: {0}")]
    Ros(String),
    #[error("Authentication failed")]
    AuthenticationFailed,
    #[error("Unsupported message type: {0}")]
    UnsupportedMessageType(String),
}

pub type Result<T> = std::result::Result<T, ServerError>;