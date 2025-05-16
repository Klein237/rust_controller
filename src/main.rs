use rust_controller::controller_server::ControllerServer;
use rclrs::{Context, SpinOptions};
use geometry_msgs::msg::Pose2D;
use std::sync::{Arc, Mutex};
use rclrs::*;
// use rust_controller::lifecycle::LifecycleState;


fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialisation ROS2
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    
    // Création du contrôleur
    let controller_server = Arc::new(Mutex::new(ControllerServer::new(&executor)?));
    controller_server.lock().unwrap().configure()?;
    controller_server.lock().unwrap().activate()?;

    // Création du subscriber avec gestion thread-safe
    let controller_clone = Arc::clone(&controller_server);
    let _subscription = controller_server.lock().unwrap().node.clone()
        .create_subscription::<Pose2D, _>(
            "/robot_pose",
            move |msg: Pose2D| {
                let mut controller = controller_clone.lock().unwrap();
                // update the pose of the robot
                controller.pose_turtle = Some(msg);

                let _ = controller.run();
            },
        )?;


    // Boucle principale
    executor.spin(SpinOptions::default());
    
    Ok(())
}