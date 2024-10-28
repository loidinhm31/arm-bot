mod def;

use actix_cors::Cors;
use actix_web::{web, App, Error, HttpResponse, HttpServer};
use chrono::{Duration, Utc};
use def::auth::{
    Claims,
    LoginCredentials,
    RefreshTokenRequest,
    User,
};
use jsonwebtoken::{decode, encode, DecodingKey, EncodingKey, Header, Validation};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::RwLock;
use uuid::Uuid;

#[derive(Debug, Serialize, Deserialize)]
pub struct TokenPair {
    access_token: String,
    refresh_token: String,
    access_expires_in: i64,
    refresh_expires_in: i64,
}

#[derive(Clone)]
pub struct AuthServer {
    jwt_secret: String,
    refresh_token_store: web::Data<RwLock<HashMap<String, String>>>, // Maps refresh_token to user_id
}

impl AuthServer {
    pub fn new() -> Self {
        let jwt_secret = env!("JWT_SECRET");
        AuthServer {
            jwt_secret: jwt_secret.to_string(),
            refresh_token_store: web::Data::new(RwLock::new(HashMap::new())),
        }
    }

    pub fn get_jwt_secret(&self) -> String {
        self.jwt_secret.clone()
    }

    fn generate_token_pair(&self, user_id: &str) -> Result<TokenPair, jsonwebtoken::errors::Error> {
        // Generate access token (short-lived)
        let access_expiration = Utc::now()
            .checked_add_signed(Duration::minutes(10))
            .expect("valid timestamp")
            .timestamp();

        let access_claims = Claims {
            sub: user_id.to_string(),
            exp: access_expiration,
            iat: Utc::now().timestamp(),
        };

        let access_token = encode(
            &Header::default(),
            &access_claims,
            &EncodingKey::from_secret(self.jwt_secret.as_bytes()),
        )?;

        // Generate refresh token (long-lived)
        let refresh_expiration = Utc::now()
            .checked_add_signed(Duration::days(1))
            .expect("valid timestamp")
            .timestamp();

        let refresh_claims = Claims {
            sub: user_id.to_string(),
            exp: refresh_expiration,
            iat: Utc::now().timestamp(),
        };

        let refresh_token = encode(
            &Header::default(),
            &refresh_claims,
            &EncodingKey::from_secret(self.jwt_secret.as_bytes()),
        )?;

        // Store refresh token
        self.refresh_token_store
            .write()
            .unwrap()
            .insert(refresh_token.clone(), user_id.to_string());

        Ok(TokenPair {
            access_token,
            refresh_token,
            access_expires_in: access_expiration - Utc::now().timestamp(),
            refresh_expires_in: refresh_expiration - Utc::now().timestamp(),
        })
    }

    fn validate_refresh_token(&self, refresh_token: &str) -> Result<String, jsonwebtoken::errors::Error> {
        // Check if refresh token exists in store
        let store = self.refresh_token_store.read().unwrap();
        if let Some(user_id) = store.get(refresh_token) {
            Ok(user_id.clone())
        } else {
            Err(jsonwebtoken::errors::Error::from(
                jsonwebtoken::errors::ErrorKind::InvalidToken,
            ))
        }
    }

    fn invalidate_refresh_token(&self, refresh_token: &str) {
        self.refresh_token_store
            .write()
            .unwrap()
            .remove(refresh_token);
    }
}

async fn authenticate(
    server: web::Data<AuthServer>,
    credentials: web::Json<LoginCredentials>,
) -> HttpResponse {
    let valid_user = User {
        username: env!("BASIC_USERNAME").to_string(),
        password: env!("BASIC_PASSWORD").to_string(),
    };

    println!("Authenticating username: {}...", &credentials.username);

    if credentials.username == valid_user.username && credentials.password == valid_user.password {
        let user_id = Uuid::new_v4().to_string();
        match server.generate_token_pair(&user_id) {
            Ok(token_pair) => HttpResponse::Ok().json(token_pair),
            Err(e) => HttpResponse::InternalServerError().json(format!("Token generation failed: {}", e)),
        }
    } else {
        HttpResponse::Unauthorized().json("Invalid credentials")
    }
}

async fn refresh_token(
    server: web::Data<AuthServer>,
    refresh_request: web::Json<RefreshTokenRequest>,
) -> HttpResponse {
    match server.validate_refresh_token(&refresh_request.refresh_token) {
        Ok(user_id) => {
            // Invalidate old refresh token
            server.invalidate_refresh_token(&refresh_request.refresh_token);

            // Generate new token pair
            match server.generate_token_pair(&user_id) {
                Ok(token_pair) => HttpResponse::Ok().json(token_pair),
                Err(e) => HttpResponse::InternalServerError().json(format!("Token generation failed: {}", e)),
            }
        }
        Err(_) => HttpResponse::Unauthorized().json("Invalid refresh token"),
    }
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    let auth_server = web::Data::new(AuthServer::new());

    println!("Starting authentication server on http://0.0.0.0:8089");

    HttpServer::new(move || {
        let cors = Cors::permissive()
            .allowed_methods(vec!["GET", "POST"])
            .allowed_headers(vec![
                actix_web::http::header::AUTHORIZATION,
                actix_web::http::header::ACCEPT,
                actix_web::http::header::CONTENT_TYPE,
            ])
            .supports_credentials()
            .max_age(3600);

        App::new()
            .wrap(cors)
            .app_data(auth_server.clone())
            .route("/auth", web::post().to(authenticate))
            .route("/refresh", web::post().to(refresh_token))
    })
        .bind("0.0.0.0:8089")?
        .run()
        .await
}