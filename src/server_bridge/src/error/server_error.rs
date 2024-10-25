use thiserror::Error;

#[derive(Debug, Error)]
pub enum ServerError {
    #[error("Authentication error: {0}")]
    Auth(String),

    #[error("ROS error: {0}")]
    Ros(String),

    #[error("WebSocket error: {0}")]
    WebSocket(#[from] tokio_tungstenite::tungstenite::Error),

    #[error("JSON error: {0}")]
    Json(#[from] serde_json::Error),

    #[error("Authentication failed")]
    AuthenticationFailed,

    #[error("Unsupported message def: {0}")]
    UnsupportedMessageType(String),
}

impl From<jsonwebtoken::errors::Error> for ServerError {
    fn from(err: jsonwebtoken::errors::Error) -> Self {
        ServerError::Auth(err.to_string())
    }
}

pub type Result<T> = std::result::Result<T, ServerError>;