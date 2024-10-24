use std::any::{Any, TypeId};
use std::sync::Arc;
use rosidl_runtime_rs::Message;

pub struct PublisherEntry {
    pub(crate) publisher: Box<dyn Any + Send + Sync>,
    pub(crate) type_id: TypeId,
    msg_type: String,
}

impl PublisherEntry {
    pub(crate) fn new<T: Message + 'static>(publisher: Arc<rclrs::Publisher<T>>) -> Self {
        PublisherEntry {
            publisher: Box::new(publisher),
            type_id: TypeId::of::<Arc<rclrs::Publisher<T>>>(),
            msg_type: std::any::type_name::<T>().to_string(),
        }
    }
}
