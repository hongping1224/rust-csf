pub mod csf;

#[cfg(test)]
mod tests {
    use super::csf::*;
    use glob::glob;
    use whitebox_lidar::LasFile;
    #[test]
    fn it_works() {
        let input_file = "test_data/clip/1629344234.734242_clip.las";
        // let input_file = "test_data/1676655424.820941.las";
        // let input_file = "./test_data/*.las";
        for entry in glob(input_file).expect("Fail to read glob pattern") {
            match entry {
                Ok(path) => {
                    let mut csf = Csf::new();
                    csf.b_sloop_smooth = true;
                    csf.cloth_resolution = 0.5;
                    csf.rigidness = 2;
                    csf.class_threshold = 0.2;
                    let input_file = path.as_path().display().to_string();
                    println!("{:?}", input_file);
                    let lasfile = match LasFile::new(&input_file, "r") {
                        Ok(lf) => lf,
                        Err(err) => panic!("Error reading file {}: {}", input_file, err),
                    };
                    let (gorund_index, _) = csf.set_point_cloud(lasfile).filter();
                    println!("ground point :{}", gorund_index.len());
                }
                Err(e) => println!("{:?}", e),
            }
        }
    }
}
