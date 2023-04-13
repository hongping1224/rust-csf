use std::time::Instant;
use whitebox_lidar::LasFile;
extern crate nalgebra as na;
use na::Matrix3x1;
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
    points: Vec<Matrix3x1<f64>>,
    bb_max: Matrix3x1<f64>,
    bb_min: Matrix3x1<f64>,
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
            bb_max: Matrix3x1::new(0.0, 0.0, 0.0),
            bb_min: Matrix3x1::new(0.0, 0.0, 0.0),
        }
    }

    pub fn set_point_cloud(mut self, reader: LasFile) -> Self {
        let start = Instant::now();
        let n_points = reader.header.number_of_points as usize;
        // self.points = Vec::with_capacity(n_points);
        // for i in 0..n_points {
        //     let xyz = reader.get_transformed_coords(i);
        //     self.points.push(Point3D::new(xyz.x, -xyz.z, xyz.y));
        // }

        self.points = vec![Matrix3x1::new(0.0, 0.0, 0.0); n_points];
        self.points.par_iter_mut().enumerate().for_each(|(i, p)| {
            let xyz = reader.get_transformed_coords(i);
            *p = Matrix3x1::new(xyz.x, -xyz.z, xyz.y);
        });
        // self.points.par_iter_mut().with_max_len((n_points/num_cpus::get())+1).enumerate().for_each(|(i,p)| {
        //     let xyz = reader.get_transformed_coords(i);
        //     *p = Point3D::new(xyz.x, -xyz.z, xyz.y);
        // });

        self.bb_max = Matrix3x1::new(
            reader.header.max_x,
            -reader.header.min_z,
            reader.header.max_y,
        );
        self.bb_min = Matrix3x1::new(
            reader.header.min_x,
            -reader.header.max_z,
            reader.header.min_y,
        );
        let dur = get_formatted_elapsed_time(start);
        println!("set point_cloud :{}", dur);
        self
    }

    pub fn filter(&mut self) -> (Vec<usize>, Vec<usize>) {
        let cloth_y_height = 0.05;
        let clothbuffer_d: f64 = 2.0;
        let origin_pos = Matrix3x1::new(
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
        let start = Instant::now();
        let cloth = &mut Cloth::new(
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
        let dur = get_formatted_elapsed_time(start);
        println!("generate cloth :{}", dur);

        let start = Instant::now();
        rasterization::Rasterization::raster_terrain(cloth, &self.points);
        let dur = get_formatted_elapsed_time(start);
        println!("rasterize :{}", dur);

        let time_step2 = self.time_step * self.time_step;
        let gravity = 0.2;

        cloth.add_force(Matrix3x1::new(0.0, -gravity, 0.0) * time_step2);

        let start = Instant::now();
        for _ in 0..self.iterations {
            let max_diff = cloth.time_step();
            cloth.terr_collision();
            if (max_diff != 0.0) && (max_diff < 0.005) {
                // early stop
                break;
            }
        }
        let dur = get_formatted_elapsed_time(start);
        println!("terr_collision :{}", dur);

        let start = Instant::now();
        if self.b_sloop_smooth {
            cloth.movable_filter();
        }
        let dur = get_formatted_elapsed_time(start);
        println!("movable_filter :{}", dur);
        let start = Instant::now();
        let (ground_index, non_ground_index) =
            c2cdist::cal_cloud_to_cloud_dist(cloth, &self.points, self.class_threshold);
        let dur = get_formatted_elapsed_time(start);
        println!("cal_cloud_to_cloud_dist :{}", dur);
        (ground_index, non_ground_index)
    }
}

pub fn get_formatted_elapsed_time(instant: Instant) -> String {
    let dur = instant.elapsed();
    let minutes = dur.as_secs() / 60;
    let sub_sec = dur.as_secs() % 60;
    let sub_milli = dur.subsec_nanos();
    if minutes > 0 {
        return format!("{}min {}.{:09}s", minutes, sub_sec, sub_milli);
    }
    format!("{}.{}s", sub_sec, sub_milli)
}
