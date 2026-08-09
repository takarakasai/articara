//! Build the かわさきロボット競技大会 ring and dump it for inspection, so its
//! dimensions can be checked against the rulebook drawing BEFORE anything is
//! built on top of them.
//!
//! Every number in `KawasakiRingCfg` was scaled off a figure, not read from
//! CAD (see that type's doc comment for which are dimensioned and which are
//! estimates). Getting them wrong is cheap to fix and expensive to discover
//! later, so this renders the field before it becomes a benchmark.
//!
//! Writes a top-down PGM of the heightfield (greyscale = elevation, so hole
//! rims and the bowl's dish are directly readable) plus the MJCF, and loads
//! the model to confirm MuJoCo accepts it and the grid transfers.
//!
//! Run: `cargo run --release --no-default-features --features mujoco \
//!   --example kawasaki_ring_check -- [out_dir]`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::{KawasakiRingCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;

    let out_dir = std::env::args().nth(1).unwrap_or_else(|| "/tmp/kawasaki_ring".into());
    std::fs::create_dir_all(&out_dir).expect("create out dir");

    let ring = KawasakiRingCfg::default();
    let (nrow, ncol) = ring.grid();
    let z_top = ring.z_top_m();
    println!("ring {:.2} m, grid {nrow}x{ncol} @ {:.1} mm, z_top {:.1} mm",
             ring.ring_m, ring.cell_m * 1000.0, z_top * 1000.0);

    let heights = ring.heights();

    // Top-down elevation map. PGM because it needs no image crate and any
    // viewer opens it; the point is to eyeball the layout, not to ship art.
    let mut pgm = format!("P2\n{ncol} {nrow}\n255\n");
    for r in (0..nrow).rev() {
        // Top row of the image is +y, so the picture matches the rulebook's
        // plan view rather than being flipped.
        for c in 0..ncol {
            let v = (heights[r * ncol + c] * 255.0).round().clamp(0.0, 255.0) as u32;
            pgm.push_str(&format!("{v} "));
        }
        pgm.push('\n');
    }
    let pgm_path = format!("{out_dir}/ring_elevation.pgm");
    std::fs::write(&pgm_path, pgm).expect("write pgm");
    println!("wrote {pgm_path}");

    // Feature heights at a few named points, as a numeric cross-check on
    // the picture: a map that looks right can still be scaled wrong.
    for (label, x, y) in [
        ("ring floor", 0.80, 0.80),
        ("bowl centre", 0.0, 0.0),
        ("bowl rim", 0.21, 0.0),
        ("round plate body", 0.48 + 0.13, 0.0),
        ("round plate hole", 0.48, 0.0),
        // The quad plate is drawn as a diamond, so a point straight above
        // its centre is on the DIAGONAL and lands between the holes -- the
        // first version of these probes had these two swapped, which read as
        // a geometry bug until the elevation map showed the plate was fine.
        ("quad plate hole", 0.0, 0.48 + 0.13),
        ("quad plate body", 0.075 / 1.4142, 0.48 + 0.075 / 1.4142),
    ] {
        println!("  {label:<20} ({x:+.3},{y:+.3}) -> {:.1} mm",
                 ring.height_at(x, y) * 1000.0);
    }

    // Load it in MuJoCo: an hfield the exporter emits but MuJoCo rejects,
    // or a grid whose size disagrees with the asset, only shows up here.
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");
    let robot = RobotModel::from_misa(&misa).expect("load namiashi");
    let opts = MjcfExportOptions {
        base_pos: Some([-0.75, -0.75, 0.30]),
        extra_asset_xml: Some(ring.asset_xml("kawasaki")),
        extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
        add_actuators: true,
        ..MjcfExportOptions::default()
    };
    let xml_path = format!("{out_dir}/model.xml");
    std::fs::write(&xml_path, articara::mjcf::export_mjcf_with_options(&robot, opts.clone()))
        .expect("write xml");
    println!("wrote {xml_path}");

    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_hfield_data("kawasaki", &heights).expect("fill hfield");
    println!("MuJoCo loaded the ring and accepted the {}-value grid.", heights.len());
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs: cargo run --features mujoco --example kawasaki_ring_check");
    std::process::exit(2);
}
