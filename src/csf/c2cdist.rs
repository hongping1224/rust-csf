use crate::csf::cloth::Cloth;
use libm::fabs;
use nalgebra::Matrix3x1;

pub fn cal_cloud_to_cloud_dist(
    cloth: &Cloth,
    points: &[Matrix3x1<f64>],
    threshold: f64,
) -> (Vec<usize>, Vec<usize>) {
    let mut ground_index = vec![];
    let mut non_ground_index = vec![];
    for (i, point) in points.iter().enumerate() {
        let pc_x = point.x;
        let pc_z = point.z;
        let delta_x = pc_x - cloth.origin_pos.x;
        let delta_z = pc_z - cloth.origin_pos.z;
        let col0 = (delta_x / cloth.step_x) as i32;
        let row0 = (delta_z / cloth.step_y) as i32;
        let col1 = col0 + 1;
        let row1 = row0;
        let col2 = col0 + 1;
        let row2 = row0 + 1;
        let col3 = col0;
        let row3 = row0 + 1;

        let subdelta_x = (delta_x - (col0 as f64) * cloth.step_x) / cloth.step_x;
        let subdelta_z = (delta_z - (row0 as f64) * cloth.step_y) / cloth.step_y;
        let p0_index = cloth.get_1d_index(col0, row0);
        let p1_index = cloth.get_1d_index(col3, row3);
        let p2_index = cloth.get_1d_index(col2, row2);
        let p3_index = cloth.get_1d_index(col1, row1);
        let fxy = cloth.particles[p0_index].pos.y * (1.0 - subdelta_x) * (1.0 - subdelta_z)
            + cloth.particles[p1_index].pos.y * (1.0 - subdelta_x) * subdelta_z
            + cloth.particles[p2_index].pos.y * subdelta_x * subdelta_z
            + cloth.particles[p3_index].pos.y * subdelta_x * (1.0 - subdelta_z);
        let height_var = fxy - point.y;
        if fabs(height_var) < threshold {
            ground_index.push(i);
        } else {
            non_ground_index.push(i);
        }
    }
    (ground_index, non_ground_index)
}
