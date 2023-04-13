pub mod csf;

#[cfg(test)]
mod tests {
    use super::csf::*;
    use whitebox_lidar::LasFile;
    #[test]
    fn test_small_file() {
        let input_file = "test_data/clip/1629344234.734242_clip.las";
        let mut csf = Csf::new();
        csf.b_sloop_smooth = true;
        csf.cloth_resolution = 0.5;
        csf.rigidness = 2;
        csf.class_threshold = 0.2;
        println!("{:?}", input_file);
        let lasfile = match LasFile::new(&input_file, "r") {
            Ok(lf) => lf,
            Err(err) => panic!("Error reading file {}: {}", input_file, err),
        };
        let (gorund_index, _) = csf.set_point_cloud(&lasfile).filter();
        assert!(282177 == gorund_index.len())
    }

    #[test]
    fn test_large_file() {
        let input_file = "test_data/1676655424.820941.las";
        let mut csf = Csf::new();
        csf.b_sloop_smooth = true;
        csf.cloth_resolution = 0.5;
        csf.rigidness = 2;
        csf.class_threshold = 0.2;
        println!("{:?}", input_file);
        let lasfile = match LasFile::new(&input_file, "r") {
            Ok(lf) => lf,
            Err(err) => panic!("Error reading file {}: {}", input_file, err),
        };
        let (gorund_index, _) = csf.set_point_cloud(&lasfile).filter();
        assert!(31956167 == gorund_index.len())
    }
}
