use criterion::{black_box, criterion_group, criterion_main, Criterion};

use csf::csf::Csf;
use whitebox_lidar::LasFile;

fn generate_csf() -> Csf {
    let mut csf = Csf::new();
    csf.b_sloop_smooth = true;
    csf.cloth_resolution = 0.5;
    csf.rigidness = 2;
    csf.class_threshold = 0.2;
    csf
}

fn set_point_cloud(lasfile: &LasFile) {
    let csf = generate_csf();
    csf.set_point_cloud(lasfile);
}

fn generate_cloth(csf: &Csf) {
    csf.generate_cloth();
}

fn find_groud_point(lasfile: &LasFile) {
    let csf = generate_csf();
    let (ground_pts, _) = csf.set_point_cloud(lasfile).filter();
    assert!(31956167 == ground_pts.len())
}

fn test_csf(c: &mut Criterion) {
    let input_file = "test_data/1676655424.820941.las";
    let lasfile = match LasFile::new(&input_file, "r") {
        Ok(lf) => lf,
        Err(err) => panic!("Error reading file {}: {}", input_file, err),
    };
    let csf = generate_csf().set_point_cloud(&lasfile);

    let mut group = c.benchmark_group("csf");
    group.sample_size(20);
    group.measurement_time(std::time::Duration::from_secs(10));
    group.bench_with_input("set point cloud", &lasfile, |b, f| {
        b.iter(|| set_point_cloud(f))
    });

    group.measurement_time(std::time::Duration::from_secs(5));
    group.bench_with_input("generate cloth", &csf, |b, f| b.iter(|| generate_cloth(&f)));

    group.sample_size(10);
    group.measurement_time(std::time::Duration::from_secs(100));
    group.bench_with_input("csf filter", &lasfile, |b, f| {
        b.iter(|| find_groud_point(f))
    });
    group.finish();
}

criterion_group!(benches, test_csf);

criterion_main!(benches);
