mod def;

use actix_cors::Cors;
use actix_web::{web, App, HttpResponse, HttpServer};
use chrono::{Duration, Utc};
use def::auth::{
    Claims,
    LoginCredentials,
    TokenResponse,
    User,
};
use jsonwebtoken::{encode, EncodingKey, Header};
use uuid::Uuid;

#[derive(Clone)]
pub struct AuthServer {
    jwt_secret: String,
}

impl AuthServer {
    pub fn new() -> Self {
        let jwt_secret = env!("JWT_SECRET");
        AuthServer {
            jwt_secret: jwt_secret.to_string(),
        }
    }

    pub fn get_jwt_secret(&self) -> String {
        self.jwt_secret.clone()
    }

    fn generate_token(&self) -> Result<TokenResponse, jsonwebtoken::errors::Error> {
        let expiration = Utc::now()
            .checked_add_signed(Duration::hours(24))
            .expect("valid timestamp")
            .timestamp();

        let claims = Claims {
            sub: Uuid::new_v4().to_string(),
            exp: expiration,
            iat: Utc::now().timestamp(),
        };

        let token = encode(
            &Header::default(),
            &claims,
            &EncodingKey::from_secret(self.jwt_secret.as_bytes()),
        )?;

        Ok(TokenResponse {
            token,
            expires_in: expiration - Utc::now().timestamp(),
        })
    }
}

async fn authenticate(
    server: web::Data<AuthServer>,
    credentials: web::Json<LoginCredentials>,
) -> HttpResponse {
    let valid_user = User {
        username: "admin".to_string(),
        password: "password123".to_string(),
    };

    println!("Authenticating username: {}...", &credentials.username);

    if credentials.username == valid_user.username && credentials.password == valid_user.password {
        match server.generate_token() {
            Ok(token_response) => HttpResponse::Ok().json(token_response),
            Err(e) => HttpResponse::InternalServerError().json(format!("Token generation failed: {}", e)),
        }
    } else {
        HttpResponse::Unauthorized().json("Invalid credentials")
    }
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    let auth_server = web::Data::new(AuthServer::new());

    println!("Starting authentication server on http://0.0.0.0:8089");

    HttpServer::new(move || {
        // Configure CORS middleware
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
    })
        .bind("0.0.0.0:8089")?
        .run()
        .await
}