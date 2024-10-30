use rclrs::{Context, Node, Publisher, Subscription};
use sensor_msgs::msg::JointState;
use trajectory_msgs::msg::JointTrajectory;
use trajectory_msgs::msg::JointTrajectoryPoint;
use std::error::Error;
use std::sync::Arc;
use tracing::{info};

struct SliderControl {
    _subscription: Arc<Subscription<JointState>>,
}

impl SliderControl {
    fn new(context: &Context) -> Result<Self, Box<dyn Error>> {
        let mut node = rclrs::create_node(&context, "arm_bot_slider_control")?;

        let arm_pub = node.create_publisher::<JointTrajectory>(
            "arm_controller/joint_trajectory",
            rclrs::QOS_PROFILE_DEFAULT,
        )?;

        let gripper_pub = node.create_publisher::<JointTrajectory>(
            "gripper_controller/joint_trajectory",
            rclrs::QOS_PROFILE_DEFAULT,
        )?;

        let subscription = node.create_subscription::<JointState, _>(
            "joint_commands",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg| Self::slider_callback(&arm_pub, &gripper_pub, msg),
        )?;

        info!("Slider Control Node started");

        rclrs::spin(node)?;

        Ok(SliderControl {
            _subscription: subscription,
        })
    }

    fn slider_callback(
        arm_pub: &Publisher<JointTrajectory>,
        gripper_pub: &Publisher<JointTrajectory>,
        msg: JointState,
    ) {
        let mut arm_controller = JointTrajectory::default();
        let mut gripper_controller = JointTrajectory::default();

        arm_controller.joint_names = vec![
            String::from("joint_1"),
            String::from("joint_2"),
            String::from("joint_3"),
        ];
        gripper_controller.joint_names = vec![String::from("joint_4")];

        let mut arm_goal = JointTrajectoryPoint::default();
        let mut gripper_goal = JointTrajectoryPoint::default();

        // Copy first 3 positions for arm
        arm_goal.positions = msg.position[..3].to_vec();
        // Copy 4th position for gripper
        gripper_goal.positions = vec![msg.position[3]];

        arm_controller.points = vec![arm_goal];
        gripper_controller.points = vec![gripper_goal];

        arm_pub.publish(&arm_controller).unwrap();
        gripper_pub.publish(&gripper_controller).unwrap();
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let ctx = Context::new(std::env::args())?;

    let _slider_control = SliderControl::new(&ctx)?;

    Ok(())
}