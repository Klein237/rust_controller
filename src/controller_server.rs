
use rclrs::{Executor, Node, Publisher,RclrsError};


use crate::path_handler::{PathGenerator, TypePath};
use crate::lifecycle::{LifecycleManager, LifecycleState};
use crate::localization::{DifferentialDriveSimulator};
use crate::config::Config;

use geometry_msgs::msg::{Twist, PoseStamped, Pose, Point, Quaternion};
use nav_msgs::msg::Path;
use std_msgs::msg::Header;
use std::sync::{Arc};
use builtin_interfaces::msg::Time;

use geometry_msgs::msg::Pose2D;

use log::{info, warn, error};



// this is the contructor of the controller server
pub struct ControllerServer {
    pub node: Arc<Node>,
        lifecycle: LifecycleManager,
        current_path: Option<Path>,
        current_pose: Option<PoseStamped>,
        controller_name_: String,
        desired_linear_vel_: f64,
        lookahead_distance_: f64,
        min_lookahead_dist_: f64,
        max_lookahead_dist_: f64,
        path_length_: f64,
        path_type_: String,
        pub_vel:  Option<Arc<Publisher<Twist>>>,
        pose_pub: Option<Arc<Publisher<PoseStamped>>>,
        path_pub: Option<Arc<Publisher<Path>>>,
        simulator: Option<DifferentialDriveSimulator>,
        pose: Option<Pose2D>,
        use_case: String,
    pub pose_turtle: Option<Pose2D>,
}

impl ControllerServer {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
            
        let node = executor.create_node("controller_server_node")
            .map_err(|e| RclrsError::from(e))?;

        let lifecycle = LifecycleManager::new();


        Ok(Self {
            node,
            lifecycle,
            current_path: None,
            current_pose: None,
            controller_name_: String::from("purepursuit"), 
            desired_linear_vel_: 0.1, 
            lookahead_distance_: 0.5, 
            min_lookahead_dist_: 0.3, 
            max_lookahead_dist_: 1.0, 
            path_length_: 0.0, 
            path_type_: "s_curve".to_string(),  // "lineare", "circle", "sinusoid" s_curve
            pub_vel: None,
            pose_pub: None,
            path_pub: None,
            simulator: None,
            pose: None,
            use_case: "turtle".to_string(),  //turtle or sim
            pose_turtle: None,
        })
    }


    // This function is used to load all the parameters
    pub fn configure(&mut self) -> Result<(), String> {
        
        self.lifecycle.transition(LifecycleState::Inactive)?;

        // this ROS method does not work with the string 
        // self.controller_name_ = self.node
        //     .declare_parameter("controller_name")
        //     .default(Arc::<str>::from("purepursuit"))
        //     .to_string();

        // ROS method to declare parameters it's working but not used
        // self.desired_linear_vel_ = self.node
        //     .declare_parameter("desired_linear_vel")
        //     .default(0.33)
        //     .mandatory()
        //     .unwrap()
        //     .get();

        let config = Config::from_yaml_file("src/rust_controller/config/params.yaml")?;

        self.controller_name_ = config.controller_name.unwrap_or_else(|| "purepursuit".to_string());
        self.desired_linear_vel_ = config.desired_linear_vel.unwrap_or(0.1);
        self.lookahead_distance_ = config.lookahead_distance.unwrap_or(0.5);
        self.min_lookahead_dist_ = config.min_lookahead_dist.unwrap_or(0.1);
        self.max_lookahead_dist_ = config.max_lookahead_dist.unwrap_or(3.0);
        self.path_length_ = config.path_length.unwrap_or(30.0);
        self.path_type_ = config.path_type.unwrap_or_else(|| "s_curve".to_string());
        self.use_case = config.use_case.unwrap_or_else(|| "turtle".to_string());

        let publisher = self.node.create_publisher::<Twist>("/turtle1/cmd_vel").unwrap();
        self.pub_vel = Some(publisher);

        let pose_publisher = self.node.create_publisher::<PoseStamped>("/pose").unwrap();
        self.pose_pub = Some(pose_publisher);

        let path_publisher = self.node.create_publisher::<Path>("/path").unwrap();
        self.path_pub = Some(path_publisher);


        self.simulator = Some(DifferentialDriveSimulator::new());
        
        if let Some(sim) = &mut self.simulator {
            sim.enable_noise(0.01);
        } else {
            
            info!("Aucun simulateur présent → enable_noise ignoré");
        }
        

        self.pose = Some(Pose2D {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        });

        self.pose_turtle = Some(Pose2D {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        });
        
        
        info!("[Lifecycle] Configured successfully");
        Ok(())
    }

    // This function is used to run the node
    // it is called in the main loop
    // it is used to update the pose and compute the velocity
    // and update the path
    pub fn run(&mut self) -> Result<(), String> {

        if self.lifecycle.get_state() != LifecycleState::Active {
            return Err("Controller is not active".to_string());

        } else {

        
        info!("[Lifecycle] Controller is active");



        if let Some(sim) = &mut self.simulator {
            self.pose = Some(sim.get_pose());
            
        } else {
            
            warn!("Aucun simulateur présent → get_pose ignoré");
        }

        self.current_pose  = Some(self.pose2d_to_posestamped("map").unwrap());

        

        let _ = self.compute_velocity().ok_or_else(|| {
            
            error!("Failed to compute velocity");
            None::<Twist>
        }).unwrap();

        self.update_path();

        }

    
        Ok(())
    }

    // This function is used to activate the node
    pub fn activate(&mut self) -> Result<(), String> {
        
        self.lifecycle.transition(LifecycleState::Active)?;
        self.get_path().ok();
        
        
        info!("[Lifecycle] Activated ");
    
        Ok(())
    }

    // This function is used to deactivate the node
    // it does not use now.
    pub fn deactivate(&mut self) -> Result<(), String> {
        
        self.lifecycle.transition(LifecycleState::Inactive)?;
        
        
        info!("[Lifecycle] Deactivated ");
        Ok(())
    }

    // This function is used to cleanup the node.
    // it does not use now.
    pub fn cleanup(&mut self) -> Result<(), String> {
        
        self.lifecycle.transition(LifecycleState::Finalized)?;
        
        
        info!("[Lifecycle] Cleaned up");

        Ok(())
    }

    // This function is used to get the current state of the lifecycle
    pub fn current_state(&self) -> LifecycleState {
        self.lifecycle.get_state()
    }

    // This function converts the pose2d to a pose stamped
    fn pose2d_to_posestamped(&self, frame_id: &str) -> Option<PoseStamped> {

        // Selection of the pose based on the use case
        // "sim" for simulation, "turtle" for turtlesim
        let pose2d = match self.use_case.as_str() {
            "sim" => self.pose.as_ref()?,
            "turtle" => self.pose_turtle.as_ref()?,
            _ => {
                
                error!("use_case inconnu : {}", self.use_case);
                return None;
            }
        };

        let theta = pose2d.theta;
        let qz = (theta / 2.0).sin();
        let qw = (theta / 2.0).cos();

        Some(PoseStamped {
            header: Header {
                frame_id: frame_id.to_string(),
                stamp: Time::default(),   
            },
            pose: Pose {
                position: Point {
                    x: pose2d.x,
                    y: pose2d.y,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: qz,
                    w: qw,
                },
            },
        })
    }

    // This function converts the quaternion to yaw
    fn quaternion_to_yaw(quat: &Quaternion) -> f64 {
       
        let siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
        let cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
        siny_cosp.atan2(cosy_cosp)
    }

    // This function is used to get the path
    fn get_path(&mut self) -> Result<(), String> {


        let path_type = match self.path_type_.as_str() {
            "lineare" => TypePath::Lineare,
            "circle" => TypePath::Circle,
            "sinusoid" => TypePath::Sinusoid,
            "s_curve" => TypePath::SCurve,
            _ => return Err("Invalid path type".to_string()),
        };

        let paths = PathGenerator::new(path_type);

        
        let path = paths.generate_path(self.path_length_);
        
        if path.is_empty() {
            
            error!("Path empty ");
            return Err("Path is empty".to_string());
        }
        else {

            let clock = self.node.get_clock().now();
            let total_nanos = clock.nsec; 

            let builtin_time = Time {
                sec: (total_nanos / 1_000_000_000) as i32,
                nanosec: (total_nanos % 1_000_000_000) as u32,
            };     
            
            let path = Path {
                header: Header {
                    frame_id: "map".to_string(),
                    stamp: builtin_time.clone(),
                },
                poses: path.iter().map(|(x, y)| PoseStamped {
                    header: Header {
                        frame_id: "map".to_string(),
                        stamp: builtin_time.clone(),
                    },
                    pose: Pose {
                        position: Point { x: *x, y: *y, z: 0.0 },
                        orientation: Quaternion::default(),
                    },
                }).collect(),
            };
            self.current_path = Some(path);
            
            info!("Path generated successfully");
            Ok(())
        }
        
    }

    fn update_path(&mut self) {
        let robot_pose = match &self.current_pose {
            Some(pose) => pose,
            None => {
                
                warn!("No pose available → update_path ignored");
                return;
            }
        };
    
        let path = match &mut self.current_path {
            Some(path) => path,
            None => {
                
                warn!("No path available → update_path ignored");
                return;
            }
        };
    
        // Threshold distance 
        let threshold = 0.5; // (m)
    
        // Remove points from the path that are too close to the robot
        path.poses.retain(|pose_stamped| {
            let px = pose_stamped.pose.position.x;
            let py = pose_stamped.pose.position.y;
            let dx = px - robot_pose.pose.position.x;
            let dy = py - robot_pose.pose.position.y;
            let distance = (dx*dx + dy*dy).sqrt();
    
            distance >= threshold
        });
    
        
        info!("[update_path] Path mis à jour : {} points restants", path.poses.len());
        // Publish the updated path
        if let Some(path_publisher) = &self.path_pub {
            path_publisher.publish(&self.current_path.clone().unwrap()).unwrap();
        }
        
        info!("[get_path] Path published");
    }

   
    
    // Pure pursuit controller
    // This function is used to compute the velocity of the robot
    fn pure_pursuit_controller(&mut self) -> Option<Twist> {

        
        let path = self.current_path.as_ref()?;
        let current_pose = self.current_pose.as_ref()?;

        if path.poses.is_empty() {
        
            error!("Empty path → stop robot");
            return None;
        }

    
        // find the target point
        let lookahead_distance = self.lookahead_distance_;
        let mut target_point = None;   
    
        for pose_stamped in &path.poses {
            let dx = pose_stamped.pose.position.x - current_pose.pose.position.x;
            let dy = pose_stamped.pose.position.y - current_pose.pose.position.y;
            let distance = (dx * dx + dy * dy).sqrt();

            // publish the pose
            if let Some(pose_publisher) = &self.pose_pub {
                pose_publisher.publish(pose_stamped).unwrap();
            }
    
            if distance >= lookahead_distance {
                target_point = Some((dx, dy));
                break;
            }
        }
    
        let (dx, dy) = target_point.unwrap_or_else(|| {
            // If no target point is found, use the last point in the path
            warn!("NOTE: No target point found, using the last point in the path");
            let last = path.poses.last().unwrap();
            (
                last.pose.position.x - current_pose.pose.position.x,
                last.pose.position.y - current_pose.pose.position.y
            )
        });
    
        // compute the heading
        let heading = Self::quaternion_to_yaw(&current_pose.pose.orientation);
        let x_r =  dx * heading.cos() + dy * heading.sin();
        let y_r = -dx * heading.sin() + dy * heading.cos();
    
        //compute the curvature and the angular velocity
        let linear_velocity = self.desired_linear_vel_;
        let curvature = if (x_r * x_r + y_r * y_r) != 0.0 {
            2.0 * y_r / (x_r * x_r + y_r * y_r)
        } else {
            0.0
        };
    
        let angular_velocity = curvature * linear_velocity;
    
        let mut cmd_vel = Twist::default();
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity.clamp(-1.0, 1.0);
    
        
        if let Some(publisher) = &self.pub_vel {
            publisher.publish(&cmd_vel).unwrap();
        }
    
        // Update the simulator with the computed velocities
        if let Some(sim) = &mut self.simulator {
            sim.update(cmd_vel.linear.x, cmd_vel.angular.z);
            
        }
    
        
        info!("[pure_pursuit] v = {}, w = {}", cmd_vel.linear.x, cmd_vel.angular.z);
        Some(cmd_vel)
    }
    
    
    // This function is used to compute the velocity of the robot
    fn compute_velocity(&mut self) -> Option<Twist> {

        // get the current path and controller
        match self.controller_name_.as_str() {
            "purepursuit" => {
                
                self.pure_pursuit_controller()
            }
            _ => {
                
                error!("Controller: unknown");
                None
            }
        }
        
    
    }
}
