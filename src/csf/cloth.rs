use crate::csf::particle::Particle;
use nalgebra::Matrix3x1;
extern crate queues;
use libm::fabs;
use queues::*;
use rayon::prelude::*;

const MAX_PARTICLE_FOR_POSTPROCESSIN: usize = 50;

fn get_particle_index(x: i32, y: i32, num_particles_width: i32) -> usize {
    (y * num_particles_width + x) as usize
}
fn make_constraint(neighbor: &mut [Vec<usize>], p1: usize, p2: usize) {
    neighbor[p1].push(p2);
    neighbor[p2].push(p1);
}

struct XY {
    x: i32,
    y: i32,
}

pub struct Cloth {
    constraint_iterations: i32,
    pub particles: Vec<Particle>,
    smooth_threshold: f64,
    height_threshold: f64,
    pub origin_pos: Matrix3x1<f64>,
    pub step_x: f64,
    pub step_y: f64,
    pub height_vals: Vec<f64>,
    pub num_particles_width: i32,
    pub num_particles_height: i32,
    pub neighbors: Vec<Vec<usize>>,
}

impl Cloth {
    pub fn new(
        _origin_pos: Matrix3x1<f64>,
        _num_particles_width: i32,
        _num_particles_height: i32,
        _step_x: f64,
        _step_y: f64,
        _smooth_threshold: f64,
        _height_threshold: f64,
        rigidness: i32,
        _time_step: f64,
    ) -> Cloth {
        let time_step2 = _time_step * _time_step;
        let mut p = Vec::with_capacity((_num_particles_width * _num_particles_height) as usize);

        for j in 0.._num_particles_height {
            for i in 0.._num_particles_width {
                let pos = Matrix3x1::new(
                    _origin_pos.x + (i as f64) * _step_x,
                    _origin_pos.y,
                    _origin_pos.z + (j as f64) * _step_y,
                );

                // insert particle in column i at j'th row
                p.push(Particle::new(pos, time_step2, i, j));
            }
        }
        let neighbors = &mut vec![
            Vec::<usize>::with_capacity(16);
            (_num_particles_height * _num_particles_width) as usize
        ];
        for x in 0.._num_particles_width {
            for y in 0.._num_particles_height {
                if x < (_num_particles_width - 1) {
                    make_constraint(
                        neighbors,
                        get_particle_index(x, y, _num_particles_width),
                        get_particle_index(x + 1, y, _num_particles_width),
                    );
                }
                if y < (_num_particles_height - 1) {
                    make_constraint(
                        neighbors,
                        get_particle_index(x, y, _num_particles_width),
                        get_particle_index(x, y + 1, _num_particles_width),
                    );
                }
                if x < (_num_particles_width - 1) && y < (_num_particles_height - 1) {
                    make_constraint(
                        neighbors,
                        get_particle_index(x, y, _num_particles_width),
                        get_particle_index(x + 1, y + 1, _num_particles_width),
                    );
                    make_constraint(
                        neighbors,
                        get_particle_index(x + 1, y, _num_particles_width),
                        get_particle_index(x, y + 1, _num_particles_width),
                    );
                }
            }
        }

        for x in 0.._num_particles_width {
            for y in 0.._num_particles_height {
                if x < (_num_particles_width - 2) {
                    make_constraint(
                        neighbors,
                        get_particle_index(x, y, _num_particles_width),
                        get_particle_index(x + 2, y, _num_particles_width),
                    );
                }
                if y < (_num_particles_height - 2) {
                    make_constraint(
                        neighbors,
                        get_particle_index(x, y, _num_particles_width),
                        get_particle_index(x, y + 2, _num_particles_width),
                    );
                }
                if x < (_num_particles_width - 2) && y < (_num_particles_height - 2) {
                    make_constraint(
                        neighbors,
                        get_particle_index(x, y, _num_particles_width),
                        get_particle_index(x + 2, y + 2, _num_particles_width),
                    );
                    make_constraint(
                        neighbors,
                        get_particle_index(x + 2, y, _num_particles_width),
                        get_particle_index(x, y + 2, _num_particles_width),
                    );
                }
            }
        }
        Cloth {
            constraint_iterations: rigidness,
            smooth_threshold: _smooth_threshold,
            height_threshold: _height_threshold,
            origin_pos: _origin_pos,
            step_x: _step_x,
            step_y: _step_y,
            num_particles_width: _num_particles_width,
            num_particles_height: _num_particles_height,
            particles: p,
            height_vals: Vec::new(),
            neighbors: neighbors.to_owned(),
        }
    }

    pub fn time_step(&mut self) -> f64 {
        let particle_count: usize = self.particles.len();

        // for i in 0..particle_count {
        //     self.particles[i].time_step();
        // }
        //#parellel
        self.particles.par_iter_mut().for_each(|p| {
            p.time_step();
        });

        // let start = Instant::now();
        for i in 0..particle_count {
            self.satisfy_particle_constraint(i);
        }

        // let dur = start.elapsed();
        // println!(
        //     "satisfy_particle_constraint {}.{:09}s",
        //     dur.as_secs(),
        //     dur.subsec_nanos()
        // );

        // let mut max_diff = 0.0;
        // for i in 0..particle_count {
        //     if self.particles[i].is_movable() {
        //         let diff = fabs(self.particles[i].old_pos.y - self.particles[i].pos.y);
        //         if diff > max_diff {
        //             max_diff = diff;
        //         }
        //     }
        // }

        // #parellel
        let particle = &self.particles;
        particle
            .into_par_iter()
            .filter(|x| x.is_movable())
            .map(|p| fabs(p.old_pos.y - p.pos.y))
            .reduce(
                || 0.0,
                |x, y| {
                    if x > y {
                        return x;
                    }
                    y
                },
            )
    }

    pub fn satisfy_particle_constraint(&mut self, i: usize) {
        for &j in self.neighbors[i].iter() {
            //    for &j in &self.neighbors[i]{
            let (selfmove, p2move) = Particle::calsatisfy_constraint(
                self.constraint_iterations as usize,
                &self.particles[i],
                &self.particles[j],
            );
            self.particles[i].offset_pos(selfmove);
            self.particles[j].offset_pos(p2move);
        }
    }

    pub fn get_size(&self) -> i32 {
        self.num_particles_height * self.num_particles_width
    }
    pub fn get_1d_index(&self, x: i32, y: i32) -> usize {
        (y * self.num_particles_width + x) as usize
    }

    // pub fn get_particle_1d(&self,index:usize)-> Particle{
    //     return self.particles[index];
    // }

    pub fn add_force(&mut self, direction: Matrix3x1<f64>) {
        for i in 0..self.particles.len() {
            self.particles[i].add_force(direction);
        }
    }
    pub fn terr_collision(&mut self) {
        // for i in 0..self.particles.len() {
        //     let v = self.particles[i].pos;
        //     if v.y < self.height_vals[i] {
        //         self.particles[i].offset_pos(Matrix3x1::new(0.0, self.height_vals[i] - v.y, 0.0));
        //         self.particles[i].make_unmovable();
        //     }
        // }
        //#parellel
        self.particles
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, p)| {
                let v = p.pos;
                if v.y < self.height_vals[i] {
                    p.offset_pos(Matrix3x1::new(0.0, self.height_vals[i] - v.y, 0.0));
                    p.make_unmovable();
                }
            })
    }
    pub fn movable_filter(&mut self) {
        for x in 0..self.num_particles_width {
            for y in 0..self.num_particles_height {
                let ptc = self.particles[self.get_1d_index(x, y)];

                if !ptc.is_movable() || ptc.is_visited {
                    continue;
                }

                let mut que: Queue<usize> = queue![];
                let mut connected: Vec<XY> = vec![]; // store the connected component
                let mut neibors: Vec<Vec<usize>> = vec![]; // store the connected component
                let mut sum = 1;
                let index = (y * self.num_particles_width + x) as usize;
                connected.push(XY { x, y });
                self.particles[index].is_visited = true;
                que.add(index).unwrap();
                while !que.size() == 0 {
                    let ptc_f = match que.remove() {
                        Ok(x) => x,
                        _ => break,
                    };
                    let cur_x = self.particles[ptc_f].pos_x;
                    let cur_y = self.particles[ptc_f].pos_y;
                    let mut neibor: Vec<usize> = vec![];
                    if cur_x > 0 {
                        let ptc_left_index = self.get_1d_index(cur_x - 1, cur_y);
                        let ptc_left = self.particles[ptc_left_index];
                        if ptc_left.is_movable() {
                            if !ptc_left.is_visited {
                                let ptc_left = &mut self.particles[ptc_left_index];
                                sum += 1;
                                ptc_left.is_visited = true;
                                connected.push(XY {
                                    x: cur_x - 1,
                                    y: cur_y,
                                });
                                que.add((self.num_particles_width * cur_y + cur_x - 1) as usize)
                                    .unwrap();
                                neibor.push(sum - 1);
                                ptc_left.c_pos = sum - 1;
                            } else {
                                neibor.push(ptc_left.c_pos);
                            }
                        }
                    }

                    if cur_x < self.num_particles_width - 1 {
                        let ptc_right_index = self.get_1d_index(cur_x + 1, cur_y);
                        let ptc_right = self.particles[ptc_right_index];
                        if ptc_right.is_movable() {
                            if !ptc_right.is_visited {
                                let mut ptc_right = &mut self.particles[ptc_right_index];
                                sum += 1;
                                ptc_right.is_visited = true;
                                connected.push(XY {
                                    x: cur_x + 1,
                                    y: cur_y,
                                });
                                que.add((self.num_particles_width * cur_y + cur_x + 1) as usize)
                                    .unwrap();
                                neibor.push(sum - 1);
                                ptc_right.c_pos = sum - 1;
                            } else {
                                neibor.push(ptc_right.c_pos);
                            }
                        }
                    }
                    if cur_y > 0 {
                        let ptc_bottom_index = self.get_1d_index(cur_x, cur_y - 1);
                        let ptc_bottom = self.particles[ptc_bottom_index];
                        if ptc_bottom.is_movable() {
                            if !ptc_bottom.is_visited {
                                let mut ptc_bottom = &mut self.particles[ptc_bottom_index];
                                sum += 1;
                                ptc_bottom.is_visited = true;
                                connected.push(XY {
                                    x: cur_x,
                                    y: cur_y - 1,
                                });
                                que.add((self.num_particles_width * (cur_y - 1) + cur_x) as usize)
                                    .unwrap();
                                neibor.push(sum - 1);
                                ptc_bottom.c_pos = sum - 1;
                            } else {
                                neibor.push(ptc_bottom.c_pos);
                            }
                        }
                    }
                    if cur_y < self.num_particles_height - 1 {
                        let ptc_top_index = self.get_1d_index(cur_x, cur_y + 1);
                        let ptc_top = self.particles[ptc_top_index];
                        if ptc_top.is_movable() {
                            if !ptc_top.is_visited {
                                let mut ptc_top = &mut self.particles[ptc_top_index];
                                sum += 1;
                                ptc_top.is_visited = true;
                                connected.push(XY {
                                    x: cur_x,
                                    y: cur_y + 1,
                                });
                                que.add((self.num_particles_width * (cur_y + 1) + cur_x) as usize)
                                    .unwrap();
                                neibor.push(sum - 1);
                                ptc_top.c_pos = sum - 1;
                            } else {
                                neibor.push(ptc_top.c_pos);
                            }
                        }
                    }
                    neibors.push(neibor);
                }
                if sum > MAX_PARTICLE_FOR_POSTPROCESSIN {
                    let edge_points = self.find_unmovable_point(&connected);
                    self.handle_slop_connected(edge_points, &connected, neibors);
                }
            }
        }
    }
    fn find_unmovable_point(&mut self, connected: &[XY]) -> Vec<usize> {
        let mut edge_points: Vec<usize> = vec![];
        for (i, xy) in connected.iter().enumerate() {
            let x = xy.x;
            let y = xy.y;
            let index = self.get_1d_index(x, y);
            let ptc = &self.particles[index];
            if x > 0 {
                let ptc_x_index = self.get_1d_index(x - 1, y);
                let ptc_x = &self.particles[ptc_x_index];

                if !ptc_x.is_movable()
                    && (fabs(self.height_vals[index] - self.height_vals[ptc_x_index])
                        < self.smooth_threshold)
                    && (ptc.pos.y - self.height_vals[index] < self.height_threshold)
                {
                    let offset_vec = Matrix3x1::new(0.0, self.height_vals[index] - ptc.pos.y, 0.0);
                    self.particles[index].offset_pos(offset_vec);
                    self.particles[index].make_unmovable();
                    edge_points.push(i);
                    continue;
                }
            }
            if x < self.num_particles_width - 1 {
                let ptc_x_index = self.get_1d_index(x + 1, y);
                let ptc_x = &self.particles[ptc_x_index];

                if !ptc_x.is_movable()
                    && (fabs(self.height_vals[index] - self.height_vals[ptc_x_index])
                        < self.smooth_threshold)
                    && (ptc.pos.y - self.height_vals[index] < self.height_threshold)
                {
                    let offset_vec = Matrix3x1::new(0.0, self.height_vals[index] - ptc.pos.y, 0.0);
                    self.particles[index].offset_pos(offset_vec);
                    self.particles[index].make_unmovable();
                    edge_points.push(i);
                    continue;
                }
            }
            if y > 0 {
                let ptc_x_index = self.get_1d_index(x, y - 1);
                let ptc_x = &self.particles[ptc_x_index];

                if !ptc_x.is_movable()
                    && (fabs(self.height_vals[index] - self.height_vals[ptc_x_index])
                        < self.smooth_threshold)
                    && (ptc.pos.y - self.height_vals[index] < self.height_threshold)
                {
                    let offset_vec = Matrix3x1::new(0.0, self.height_vals[index] - ptc.pos.y, 0.0);
                    self.particles[index].offset_pos(offset_vec);
                    self.particles[index].make_unmovable();
                    edge_points.push(i);
                    continue;
                }
            }
            if y < self.num_particles_height - 1 {
                let ptc_x_index = self.get_1d_index(x, y + 1);
                let ptc_x = &self.particles[ptc_x_index];

                if !ptc_x.is_movable()
                    && (fabs(self.height_vals[index] - self.height_vals[ptc_x_index])
                        < self.smooth_threshold)
                    && (ptc.pos.y - self.height_vals[index] < self.height_threshold)
                {
                    let offset_vec = Matrix3x1::new(0.0, self.height_vals[index] - ptc.pos.y, 0.0);
                    self.particles[index].offset_pos(offset_vec);
                    self.particles[index].make_unmovable();
                    edge_points.push(i);
                    continue;
                }
            }
        }
        edge_points
    }

    fn handle_slop_connected(
        &mut self,
        edge_points: Vec<usize>,
        connected: &Vec<XY>,
        neibors: Vec<Vec<usize>>,
    ) {
        let mut visited: Vec<bool> = vec![false; connected.len()];
        let mut que: Queue<usize> = queue![];

        for i in 0..edge_points.len() {
            que.add(edge_points[i]).unwrap();
            visited[edge_points[i]] = true;
        }
        while !que.size() == 0 {
            let index = match que.remove() {
                Ok(x) => x,
                _ => break,
            };

            let index_center = self.get_1d_index(connected[index].x, connected[index].y);

            for i in 0..neibors[index].len() {
                let index_neibor = self.get_1d_index(
                    connected[neibors[index][i]].x,
                    connected[neibors[index][i]].y,
                );

                if (fabs(self.height_vals[index_center] - self.height_vals[index_neibor])
                    < self.smooth_threshold)
                    && (fabs(self.particles[index_neibor].pos.y - self.height_vals[index_neibor])
                        < self.height_threshold)
                {
                    let offset_vec = Matrix3x1::new(
                        0.0,
                        self.height_vals[index_neibor] - self.particles[index_neibor].pos.y,
                        0.0,
                    );
                    self.particles[index_neibor].offset_pos(offset_vec);
                    self.particles[index_neibor].make_unmovable();

                    if !visited[neibors[index][i]] {
                        que.add(neibors[index][i]).unwrap();
                        visited[neibors[index][i]] = true;
                    }
                }
            }
        }
    }
}
