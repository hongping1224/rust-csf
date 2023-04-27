use whitebox_lidar::LasFile;
extern crate nalgebra as na;
use na::Vector3;
mod c2cdist;
mod cloth;
mod particle;
mod rasterization;
use crate::csf::cloth::Cloth;
use rayon::prelude::*;

pub struct Csf {
    pub b_sloop_smooth: bool,
    pub time_step: f64,
    pub class_threshold: f64,
    pub cloth_resolution: f64,
    pub rigidness: i32,
    pub iterations: i32,
    pub points: Vec<Vector3<f64>>,
    bb_max: Vector3<f64>,
    bb_min: Vector3<f64>,
}

impl Default for Csf {
    fn default() -> Self {
        Self::new()
    }
}

impl Csf {
    pub fn new() -> Csf {
        Csf {
            b_sloop_smooth: true,
            time_step: 0.65,
            class_threshold: 0.5,
            cloth_resolution: 1.0,
            rigidness: 3,
            iterations: 500,
            points: Vec::new(),
            bb_max: Vector3::new(0.0, 0.0, 0.0),
            bb_min: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    pub fn set_point_cloud_with_index(mut self, reader: &LasFile, index: &Vec<usize>) -> Self {
        let n_points: usize = index.len();
        self.points = vec![Vector3::new(0.0, 0.0, 0.0); n_points];
        self.points.par_iter_mut().enumerate().for_each(|(i, p)| {
            let xyz = reader.get_transformed_coords(index[i]);
            *p = Vector3::new(xyz.x, -xyz.z, xyz.y);
        });

        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        let mut max_z = f64::MIN;
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut min_z = f64::MAX;
        self.points.iter().for_each(|&p| {
            max_x = max_x.max(p.x);
            max_y = max_y.max(p.y);
            max_z = max_z.max(p.z);
            min_x = min_x.min(p.x);
            min_y = min_y.min(p.y);
            min_z = min_z.min(p.z);
        });
        self.bb_max = Vector3::new(max_x, max_y, max_z);
        self.bb_min = Vector3::new(min_x, min_y, min_z);
        self
    }
    pub fn set_point_cloud(mut self, reader: &LasFile) -> Self {
        let n_points = reader.header.number_of_points as usize;
        // self.points = Vec::with_capacity(n_points);
        // for i in 0..n_points {
        //     let xyz = reader.get_transformed_coords(i);
        //     self.points.push(Point3D::new(xyz.x, -xyz.z, xyz.y));
        // }

        self.points = vec![Vector3::new(0.0, 0.0, 0.0); n_points];
        self.points.par_iter_mut().enumerate().for_each(|(i, p)| {
            let xyz = reader.get_transformed_coords(i);
            *p = Vector3::new(xyz.x, -xyz.z, xyz.y);
        });

        self.bb_max = Vector3::new(
            reader.header.max_x,
            -reader.header.min_z,
            reader.header.max_y,
        );
        self.bb_min = Vector3::new(
            reader.header.min_x,
            -reader.header.max_z,
            reader.header.min_y,
        );
        self
    }

    pub fn generate_cloth(&self) -> Cloth {
        let cloth_y_height = 0.05;
        let clothbuffer_d: f64 = 2.0;
        let origin_pos = Vector3::new(
            self.bb_min.x - (clothbuffer_d * self.cloth_resolution),
            self.bb_max.y + cloth_y_height,
            self.bb_min.z - (clothbuffer_d * self.cloth_resolution),
        );

        let width_num: i32 =
            ((self.bb_max.x - self.bb_min.x) / self.cloth_resolution).floor() as i32;
        let width_num: i32 = width_num + 2 * (clothbuffer_d as i32);

        let height_num: i32 =
            ((self.bb_max.z - self.bb_min.z) / self.cloth_resolution).floor() as i32;
        let height_num: i32 = height_num + 2 * (clothbuffer_d as i32);
        let cloth = Cloth::new(
            origin_pos,
            width_num,
            height_num,
            self.cloth_resolution,
            self.cloth_resolution,
            0.3,
            9999.0,
            self.rigidness,
            self.time_step,
        );
        cloth
    }

    pub fn filter(&mut self) -> (Vec<usize>, Vec<usize>) {
        let cloth = &mut self.generate_cloth();

        rasterization::Rasterization::raster_terrain(cloth, &self.points);

        let time_step2 = self.time_step * self.time_step;
        let gravity = 0.2;

        cloth.add_force(Vector3::new(0.0, -gravity, 0.0) * time_step2);

        for _ in 0..self.iterations {
            let max_diff = cloth.time_step();
            cloth.terr_collision();
            if (max_diff != 0.0) && (max_diff < 0.005) {
                // early stop
                break;
            }
        }

        if self.b_sloop_smooth {
            cloth.movable_filter();
        }

        let (ground_index, non_ground_index) =
            c2cdist::cal_cloud_to_cloud_dist(cloth, &self.points, self.class_threshold);

        (ground_index, non_ground_index)
    }
}

// pub fn get_formatted_elapsed_time(instant: Instant) -> String {
//     let dur = instant.elapsed();
//     let minutes = dur.as_secs() / 60;
//     let sub_sec = dur.as_secs() % 60;
//     let sub_milli = dur.subsec_nanos();
//     if minutes > 0 {
//         return format!(
//             "{}min {}s",
//             minutes,
//             sub_sec as f64 + (sub_milli) as f64 / 1_000_000_000.0
//         );
//     }
//     format!("{}s", sub_sec as f64 + (sub_milli) as f64 / 1_000_000_000.0)
// }
