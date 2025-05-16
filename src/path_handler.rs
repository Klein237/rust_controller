
#[derive(Debug,Clone, Copy)]
pub enum TypePath {
    Lineare,
    Circle,
    Sinusoid,
}

pub struct PathGenerator {
   pathef: TypePath,
}

impl PathGenerator {
    pub fn new(path: TypePath) -> Self {
        Self { pathef: path }
    }

    pub fn generate_path(&self, length: f64) -> Vec<(f64, f64)> {
        match self.pathef {
            TypePath::Lineare => self.generate_lineare_path(length),
            TypePath::Circle => self.generate_circle_path(length),
            TypePath::Sinusoid => self.generate_sinusoid_path(length),
        }
    }

    fn generate_lineare_path(&self, length: f64) -> Vec<(f64, f64)> {
        let mut path = Vec::new();
        for i in 0..(length as usize) {
            path.push((i as f64, 0.0));
        }
        path
    }
    fn generate_circle_path(&self, length: f64) -> Vec<(f64, f64)> {
        let mut path = Vec::new();
        let radius = length / (2.0 * std::f64::consts::PI);
        for i in 0..(length as usize) {
            let angle = i as f64 * (2.0 * std::f64::consts::PI / length);
            path.push((radius * angle.cos(), radius * angle.sin()));
        }
        path
    }
    fn generate_sinusoid_path(&self, length: f64) -> Vec<(f64, f64)> {
        let mut path = Vec::new();
        for i in 0..(length as usize) {
            path.push((i as f64, (i as f64).sin()));
        }
        path
    }
    pub fn set_path(&mut self, path: TypePath) {
        self.pathef = path;
    }
    pub fn get_path(&self) -> TypePath {
        self.pathef
    }
    
}