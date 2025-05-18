use serde::Deserialize;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

#[derive(Debug, Deserialize)]
pub struct Config {
    pub controller_name: Option<String>,
    pub desired_linear_vel: Option<f64>,
    pub lookahead_distance: Option<f64>,
    pub min_lookahead_dist: Option<f64>,
    pub max_lookahead_dist: Option<f64>,
    pub path_length: Option<f64>,
    pub path_type: Option<String>,
    pub use_case: Option<String>,
}




impl Config {
    pub fn from_yaml_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let file = File::open(path).map_err(|e| format!("Erreur ouverture YAML : {}", e))?;
        let reader = BufReader::new(file);
        serde_yaml::from_reader(reader).map_err(|e| format!("Erreur parsing YAML : {}", e))
    }
}

