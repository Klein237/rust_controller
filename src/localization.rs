use std::time::{Instant};
use rand::Rng;
use geometry_msgs::msg::Pose2D;

// structure of the simulator
pub struct DifferentialDriveSimulator {
    pose: Pose2D,
    last_update_time: Option<Instant>,
    noise_enabled: bool,
    noise_std_dev: f64,
}

impl DifferentialDriveSimulator {
    pub fn new() -> Self {
        Self {
            pose: Pose2D {
                x: 0.0,
                y: 0.0,
                theta: 0.0,
            },
            last_update_time: None,
            noise_enabled: false,
            noise_std_dev: 0.0,
        }
    }

    // enable the gaussian noise
    pub fn enable_noise(&mut self, std_dev: f64) {
        self.noise_enabled = true;
        self.noise_std_dev = std_dev;
        
    }

    // Update the pose of the robot
    // according to the linear and angular velocities
    pub fn update(&mut self, linear_velocity: f64, angular_velocity: f64)  {
        let now = Instant::now();
        let dt = if let Some(last_time) = self.last_update_time {
            now.duration_since(last_time).as_secs_f64()
        } else {
            0.0
        };
        self.last_update_time = Some(now);

        if dt == 0.0 {
            return; 
        }

        let mut v = linear_velocity;
        let mut w = angular_velocity;

        // Add noise to the velocities if enabled
        if self.noise_enabled {
            let mut rng = rand::thread_rng();
            v += rng.gen_range(-self.noise_std_dev..self.noise_std_dev);
            w += rng.gen_range(-self.noise_std_dev..self.noise_std_dev);
        }

        // update the pose of the robot using the differential drive model
        self.pose.x += v * dt * self.pose.theta.cos();
        self.pose.y += v * dt * self.pose.theta.sin();
        self.pose.theta += w * dt;

        //[-pi, pi]
        self.pose.theta = (self.pose.theta + std::f64::consts::PI) % (2.0 * std::f64::consts::PI) - std::f64::consts::PI;

      
    }

    /// Get the current pose of the robot
    pub fn get_pose(&self) -> Pose2D {
        self.pose.clone()
    }

    /// Reset the pose of the robot to the initial position
    pub fn reset(&mut self) {
        self.pose = Pose2D {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        self.last_update_time = None;
    }
}
