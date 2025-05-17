
#[derive(Debug,Clone, Copy)]
pub enum TypePath {
    Lineare,
    Circle,
    Sinusoid,
    SCurve,
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
            TypePath::SCurve => self.generate_s_curve_path(length),
        }
    }

    fn generate_lineare_path(&self, length: f64) -> Vec<(f64, f64)> {
        let mut path = Vec::new();
        for i in 1..(length as usize) {
            let y = i as f64;
            path.push((i as f64, y));
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
        for i in 1..(length as usize) {
            path.push((i as f64, (i as f64).cos()));
        }
        path
    }

    pub fn generate_s_curve_path(&self, length: f64) -> Vec<(f64, f64)> {
        let mut path = Vec::new();
    
        let step = 0.1;
        let a = 2.0; 
        let b = 1.0; 
        let f = 0.2; 
        let g = 0.6; 
    
        let mut x = 0.0;
        let mut min_y = f64::MAX;
        let mut temp_path = Vec::new();
    
        
        while x < length {
            let y = a * (f * x).sin() + b * (g * x).sin();
            if y < min_y {
                min_y = y;
            }
            temp_path.push((x, y));
            x += step;
        }
    
        
        for (x, y) in temp_path {
            let new_x = x + 1.0;
            let new_y = y + (1.0 - min_y);  
            path.push((new_x, new_y));
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