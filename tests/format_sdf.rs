//! SDF import / export (split from regression.rs).
//!
//! Shared fixture paths live in `common::fixtures`.

mod common;

#[allow(unused_imports)]
use common::fixtures::*;
#[allow(unused_imports)]
use std::path::{Path, PathBuf};

// ============================================================
// sdf.rs — SDF import and export
// ============================================================
mod test_sdf {
    use super::*;
    use articara::robot::*;
    use articara::sdf;

    #[test]
    fn import_sdf_basic() {
        let model = sdf::import_sdf(&fixture_sdf()).expect("SDF import failed");
        assert_eq!(model.name, "test_sdf_robot");
        assert_eq!(model.links.len(), 3);
        assert_eq!(model.joints.len(), 2);
        assert_eq!(model.root_link, "base_link");
    }

    #[test]
    fn sdf_link_inertial() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert!((base.inertial.mass - 1.0).abs() < 1e-6);
        assert!((base.inertial.ixx - 0.01).abs() < 1e-6);
    }

    #[test]
    fn sdf_visual_geometry() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert_eq!(base.visuals.len(), 1);
        match &base.visuals[0].geometry {
            GeomData::Box { hx, hy, hz } => {
                assert!((hx - 0.1).abs() < 1e-4);
                assert!((hy - 0.1).abs() < 1e-4);
                assert!((hz - 0.05).abs() < 1e-4);
            }
            _ => panic!("Expected Box geometry"),
        }
    }

    #[test]
    fn sdf_cylinder_geometry() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let l1 = model.links.iter().find(|l| l.name == "link1").unwrap();
        match &l1.visuals[0].geometry {
            GeomData::Cylinder { radius, half_length } => {
                assert!((*radius - 0.02).abs() < 1e-4);
                assert!((*half_length - 0.1).abs() < 1e-4);
            }
            _ => panic!("Expected Cylinder geometry"),
        }
    }

    #[test]
    fn sdf_sphere_geometry() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let l2 = model.links.iter().find(|l| l.name == "link2").unwrap();
        match &l2.visuals[0].geometry {
            GeomData::Sphere { radius } => {
                assert!((*radius - 0.03).abs() < 1e-4);
            }
            _ => panic!("Expected Sphere geometry"),
        }
    }

    #[test]
    fn sdf_joint_properties() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let j1 = model.joints.iter().find(|j| j.name == "joint1").unwrap();
        assert_eq!(j1.joint_type, "revolute");
        assert_eq!(j1.parent_link, "base_link");
        assert_eq!(j1.child_link, "link1");
        assert!((j1.lower - (-1.57)).abs() < 1e-4);
        assert!((j1.upper - 1.57).abs() < 1e-4);
    }

    #[test]
    fn sdf_visual_color() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let l1 = model.links.iter().find(|l| l.name == "link1").unwrap();
        let c = l1.visuals[0].color;
        assert!((c[0] - 1.0).abs() < 1e-3); // red=1
        assert!((c[1]).abs() < 1e-3);         // green=0
    }

    #[test]
    fn sdf_collision_parsed() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
        assert_eq!(base.collisions.len(), 1);
    }

    #[test]
    fn sdf_source_path() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        assert!(model.source_path.is_some());
    }

    #[test]
    fn export_sdf_contains_model() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let xml = sdf::export_sdf(&model);
        assert!(xml.contains("<sdf"));
        assert!(xml.contains("<model name=\"test_sdf_robot\""));
        assert!(xml.contains("<link name=\"base_link\""));
        assert!(xml.contains("<joint name=\"joint1\""));
        assert!(xml.contains("<mass>"));
    }

    #[test]
    fn sdf_roundtrip_data_preserved() {
        let model = sdf::import_sdf(&fixture_sdf()).unwrap();
        let xml = sdf::export_sdf(&model);

        // Write to temp, re-import
        let tmp = std::env::temp_dir().join("articara_test_sdf_roundtrip.sdf");
        std::fs::write(&tmp, &xml).unwrap();
        let model2 = sdf::import_sdf(&tmp).expect("Re-import SDF failed");
        std::fs::remove_file(&tmp).ok();

        assert_eq!(model.name, model2.name);
        assert_eq!(model.links.len(), model2.links.len());
        assert_eq!(model.joints.len(), model2.joints.len());
        assert_eq!(model.root_link, model2.root_link);

        // Check inertial mass round-trips
        for (a, b) in model.links.iter().zip(model2.links.iter()) {
            assert!((a.inertial.mass - b.inertial.mass).abs() < 1e-4,
                "Mass mismatch for {}: {} vs {}", a.name, a.inertial.mass, b.inertial.mass);
        }
    }

    // ---- Closed-loop models ----

    fn fixture_four_bar() -> PathBuf {
        fixtures_dir().join("sdf").join("four_bar.sdf")
    }

    fn fixture_five_bar() -> PathBuf {
        fixtures_dir().join("sdf").join("five_bar_parallel.sdf")
    }

    #[test]
    fn four_bar_loads() {
        let model = sdf::import_sdf(&fixture_four_bar()).expect("four_bar import failed");
        assert_eq!(model.name, "four_bar_linkage");
        assert_eq!(model.links.len(), 4);
        assert_eq!(model.joints.len(), 3);
        assert_eq!(model.root_link, "base_link");
    }

    #[test]
    fn four_bar_loop_closed_at_q0() {
        let model = sdf::import_sdf(&fixture_four_bar()).unwrap();
        let transforms = model.compute_transforms();

        // Coupler tip = coupler origin + (0.3, 0, 0) in coupler frame
        let coupler_tf = transforms["coupler"];
        let coupler_tip = coupler_tf * nalgebra::Point3::new(0.3_f32, 0.0, 0.0);

        // Crank-right tip = crank_right origin + (0, 0, 0.2) in crank_right frame
        let crank_r_tf = transforms["crank_right"];
        let crank_r_tip = crank_r_tf * nalgebra::Point3::new(0.0_f32, 0.0, 0.2);

        let err = (coupler_tip - crank_r_tip).norm();
        assert!(
            err < 0.01,
            "Four-bar loop not closed at q=0: coupler_tip={:?} crank_r_tip={:?} err={}",
            coupler_tip, crank_r_tip, err
        );
    }

    #[test]
    fn five_bar_loads() {
        let model = sdf::import_sdf(&fixture_five_bar()).expect("five_bar import failed");
        assert_eq!(model.name, "five_bar_parallel");
        assert_eq!(model.links.len(), 6);  // base + 2 proximal + 2 distal + EE
        // 4 revolute + 1 fixed = 5 joints
        assert_eq!(model.joints.len(), 5);
        assert_eq!(model.root_link, "base_link");
    }

    #[test]
    fn five_bar_loop_closed_at_q0() {
        let model = sdf::import_sdf(&fixture_five_bar()).unwrap();
        let transforms = model.compute_transforms();

        // EE position (fixed to distal_left tip)
        let ee_pos = transforms["end_effector"]
            * nalgebra::Point3::new(0.0_f32, 0.0, 0.0);

        // Distal-right tip = distal_right origin + (0, 0, 0.2) in its frame
        let dr_tf = transforms["distal_right"];
        let dr_tip = dr_tf * nalgebra::Point3::new(0.0_f32, 0.0, 0.2);

        let err = (ee_pos - dr_tip).norm();
        assert!(
            err < 0.01,
            "Five-bar loop not closed at q=0: ee={:?} dr_tip={:?} err={}",
            ee_pos, dr_tip, err
        );
    }
}
