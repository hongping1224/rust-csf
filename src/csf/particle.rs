use super::na::Matrix3x1;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Particle {
    movable: bool,
    mass: f64,
    acceleration: Matrix3x1<f64>,
    time_step: f64,
    pub pos: Matrix3x1<f64>,
    pub old_pos: Matrix3x1<f64>,
    pub is_visited: bool,
    pub pos_x: i32,
    pub pos_y: i32,
    pub c_pos: usize,
    pub nearest_point_index: usize,
    pub nearest_point_height: f64,
    pub tmp_dist: f64,
}

const SINGLE_MOVE1: [f64; 15] = [
    0.0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765, 0.94235, 0.95965, 0.97175, 0.98023,
    0.98616, 0.99031, 0.99322,
];
const DOUBLE_MOVE1: [f64; 15] = [
    0.0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498, 0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5,
    0.5,
];
const DAMPING: f64 = 0.01;
impl Particle {
    pub fn new(pos: Matrix3x1<f64>, time_step: f64, x: i32, y: i32) -> Particle {
        Particle {
            movable: true,
            mass: 1.0,
            acceleration: Matrix3x1::new(0.0, 0.0, 0.0),
            time_step,
            pos,
            old_pos: pos,
            is_visited: false,
            pos_x: x,
            pos_y: y,
            c_pos: 0,
            nearest_point_index: 0,
            nearest_point_height: std::f64::MIN,
            tmp_dist: std::f64::MAX,
        }
    }

    pub fn calsatisfy_constraint(
        constraint_times: usize,
        p1: &Particle,
        p2: &Particle,
    ) -> (Matrix3x1<f64>, Matrix3x1<f64>) {
        let correction_vector = Matrix3x1::new(0.0, p2.pos.y - p1.pos.y, 0.0);
        if p1.is_movable() && p2.is_movable() {
            let correction_vector_half: Matrix3x1<f64> = correction_vector
                * (if constraint_times > 14 {
                    0.5
                } else {
                    DOUBLE_MOVE1[constraint_times]
                });
            return (correction_vector_half, -correction_vector_half);
        } else if p1.is_movable() && !p2.is_movable() {
            let correction_vector_half: Matrix3x1<f64> = correction_vector
                * (if constraint_times > 14 {
                    1.0
                } else {
                    SINGLE_MOVE1[constraint_times]
                });
            return (correction_vector_half, Matrix3x1::new(0.0, 0.0, 0.0));
        } else if !p1.is_movable() && p2.is_movable() {
            let correction_vector_half: Matrix3x1<f64> = correction_vector
                * (if constraint_times > 14 {
                    1.0
                } else {
                    SINGLE_MOVE1[constraint_times]
                });
            return (Matrix3x1::new(0.0, 0.0, 0.0), -correction_vector_half);
        }
        (Matrix3x1::new(0.0, 0.0, 0.0), Matrix3x1::new(0.0, 0.0, 0.0))
    }
    pub fn is_movable(&self) -> bool {
        self.movable
    }
    pub fn add_force(&mut self, f: Matrix3x1<f64>) {
        self.acceleration += f / self.mass;
    }
    pub fn time_step(&mut self) {
        if self.is_movable() {
            let tmp = self.pos.clone();
            self.pos = self.pos
                + (self.pos - self.old_pos) * (1.0 - DAMPING)
                + self.acceleration * self.time_step;
            self.old_pos = tmp;
        }
    }

    pub fn offset_pos(&mut self, v: Matrix3x1<f64>) {
        if self.movable {
            self.pos += v;
        }
    }
    pub fn make_unmovable(&mut self) {
        self.movable = false;
    }
}
