use std::time::{Instant};
use rand::Rng;
use geometry_msgs::msg::Pose2D;

// #[derive(Debug, Clone, Copy)]
// pub struct Pose2D {
//     pub x: f64,
//     pub y: f64,
//     pub theta: f64, // orientation en radians
// }

// impl Pose2D {
//     pub fn new() -> Self {
//         Pose2D { x: 0.0, y: 0.0, theta: 0.0 }
//     }
// }

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

    /// Active un bruit gaussien pour simuler des erreurs capteurs
    pub fn enable_noise(&mut self, std_dev: f64) {
        self.noise_enabled = true;
        self.noise_std_dev = std_dev;
        
    }

    /// Mise à jour de la position en fonction des vitesses reçues
    pub fn update(&mut self, linear_velocity: f64, angular_velocity: f64)  {
        let now = Instant::now();
        let dt = if let Some(last_time) = self.last_update_time {
            now.duration_since(last_time).as_secs_f64()
        } else {
            0.0
        };
        self.last_update_time = Some(now);

        if dt == 0.0 {
            return; // première boucle, pas de mise à jour
        }

        let mut v = linear_velocity;
        let mut w = angular_velocity;

        // Ajout d'un bruit si activé
        if self.noise_enabled {
            let mut rng = rand::thread_rng();
            v += rng.gen_range(-self.noise_std_dev..self.noise_std_dev);
            w += rng.gen_range(-self.noise_std_dev..self.noise_std_dev);
        }

        // Mise à jour de la position selon le modèle de robot différentiel
        self.pose.x += v * dt * self.pose.theta.cos();
        self.pose.y += v * dt * self.pose.theta.sin();
        self.pose.theta += w * dt;

        // Normaliser theta dans [-pi, pi]
        self.pose.theta = (self.pose.theta + std::f64::consts::PI) % (2.0 * std::f64::consts::PI) - std::f64::consts::PI;

      
    }

    /// Récupérer la pose courante
    pub fn get_pose(&self) -> Pose2D {
        self.pose.clone()
    }

    /// Réinitialiser la simulation
    pub fn reset(&mut self) {
        self.pose = Pose2D {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        self.last_update_time = None;
    }
}
