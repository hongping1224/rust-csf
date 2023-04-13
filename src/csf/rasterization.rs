use crate::csf::cloth::Cloth;
use std::ops::{Add, Mul, Sub};
use whitebox_common::structures::Point3D;
extern crate queues;

use queues::*;

pub struct Rasterization;

fn square_dist<T>(x1: T, y1: T, x2: T, y2: T) -> T
where
    T: Add<Output = T> + Sub<Output = T> + Mul<Output = T> + Copy,
{
    ((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2))
}

impl Rasterization {
    pub fn find_height_by_neighbor(index: usize, cloth: &mut Cloth) -> f64 {
        let mut n_q: Queue<usize> = queue![];
        let mut p_back_list: Vec<usize> = vec![];
        let neighbor = &cloth.neighbors[index];
        let particles = &mut cloth.particles;
        for &i in neighbor.iter() {
            particles[index].is_visited = true;
            n_q.add(i).expect("fail to add point to queue");
        }
        while n_q.size() != 0 {
            let pindex = match n_q.remove() {
                Ok(x) => x,
                _ => break,
            };
            let pneighbor_nearest_point_height = particles[pindex].nearest_point_height;
            p_back_list.push(pindex);

            if pneighbor_nearest_point_height > f64::MIN {
                for i in 0..(p_back_list.len()) {
                    particles[p_back_list[i]].is_visited = false;
                }
                while n_q.size() != 0 {
                    match n_q.remove() {
                        Ok(x) => particles[x].is_visited = false,
                        _ => continue,
                    };
                }
                return pneighbor_nearest_point_height;
            } else {
                let nsize = cloth.neighbors[pindex].len();

                for i in 0..nsize {
                    let ptmp_index = cloth.neighbors[pindex][i];
                    if !particles[ptmp_index].is_visited {
                        particles[ptmp_index].is_visited = true;
                        n_q.add(ptmp_index).unwrap();
                    }
                }
            }
        }
        f64::MIN
    }

    pub fn find_height_by_scanline(index: usize, cloth: &mut Cloth) -> f64 {
        let p = &cloth.particles[index];
        let xpos = p.pos_x;
        let ypos = p.pos_y;
        for i in (xpos + 1)..(cloth.num_particles_width) {
            let particle_index = cloth.get_1d_index(i, ypos);
            let crres_height = cloth.particles[particle_index].nearest_point_height;
            if crres_height > f64::MIN {
                return crres_height;
            }
        }
        for i in (0..=(xpos - 1)).rev() {
            let particle_index = cloth.get_1d_index(i, ypos);
            let crres_height = cloth.particles[particle_index].nearest_point_height;
            if crres_height > f64::MIN {
                return crres_height;
            }
        }

        for j in (0..=(ypos - 1)).rev() {
            let particle_index = cloth.get_1d_index(xpos, j);
            let crres_height = cloth.particles[particle_index].nearest_point_height;
            if crres_height > f64::MIN {
                return crres_height;
            }
        }
        for j in (ypos + 1)..(cloth.num_particles_height) {
            let particle_index = cloth.get_1d_index(xpos, j);
            let crres_height = cloth.particles[particle_index].nearest_point_height;
            if crres_height > f64::MIN {
                return crres_height;
            }
        }
        Self::find_height_by_neighbor(index, cloth)
    }

    pub fn raster_terrain(cloth: &mut Cloth, pc: &[Point3D]) {
        for (i, point) in pc.iter().enumerate() {
            let pc_x = point.x;
            let pc_z = point.z;

            let delta_x = pc_x - cloth.origin_pos.x;
            let delta_z = pc_z - cloth.origin_pos.z;
            let col = (delta_x / cloth.step_x + 0.5) as i32;
            let row = (delta_z / cloth.step_y + 0.5) as i32;

            if col >= 0 && row >= 0 {
                let pindex = cloth.get_1d_index(col, row);
                let p = &mut cloth.particles[pindex];
                // p.corresponding_lidar_point_list.push(i);
                let pc2particle_dist = square_dist(pc_x, pc_z, p.pos.x, p.pos.z);
                if pc2particle_dist < p.tmp_dist {
                    p.tmp_dist = pc2particle_dist;
                    p.nearest_point_height = point.y;
                    p.nearest_point_index = i;
                }
            }
        }
        let mut height_val = Vec::with_capacity(cloth.get_size() as usize);
        for i in 0..(cloth.get_size() as usize) {
            let nearest_height = cloth.particles[i].nearest_point_height;
            if nearest_height > f64::MIN {
                height_val.push(nearest_height);
            } else {
                height_val.push(Self::find_height_by_scanline(i, cloth))
            }
        }

        cloth.height_vals = height_val;
    }
}
