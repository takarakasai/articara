//! Inertia computation / validation and mesh I/O regressions (split from regression.rs).
//!
//! Shared fixture paths live in `common::fixtures`.

mod common;

#[allow(unused_imports)]
use common::fixtures::*;
#[allow(unused_imports)]
use std::path::{Path, PathBuf};

// =========================================================================
//  Inertia computation tests
// =========================================================================

mod test_inertia {
    use articara::robot::*;

    #[test]
    fn box_inertia_values() {
        // 1kg box with half-extents 0.1, 0.2, 0.3  (full: 0.2, 0.4, 0.6)
        let geom = GeomData::Box { hx: 0.1, hy: 0.2, hz: 0.3 };
        let i = compute_geometry_inertia(&geom, 1.0);
        // Ixx = m/12 * (b² + c²) = (0.4² + 0.6²) / 12 = 0.0433..
        let expected_ixx = (0.4f64.powi(2) + 0.6f64.powi(2)) / 12.0;
        let expected_iyy = (0.2f64.powi(2) + 0.6f64.powi(2)) / 12.0;
        let expected_izz = (0.2f64.powi(2) + 0.4f64.powi(2)) / 12.0;
        assert!((i.ixx - expected_ixx).abs() < 1e-6, "ixx={}", i.ixx);
        assert!((i.iyy - expected_iyy).abs() < 1e-6, "iyy={}", i.iyy);
        assert!((i.izz - expected_izz).abs() < 1e-6, "izz={}", i.izz);
        assert!((i.ixy).abs() < 1e-10);
        assert!((i.ixz).abs() < 1e-10);
        assert!((i.iyz).abs() < 1e-10);
    }

    #[test]
    fn cylinder_inertia_values() {
        // 2kg cylinder, radius=0.05, half_length=0.1  (height=0.2)
        let geom = GeomData::Cylinder { radius: 0.05, half_length: 0.1 };
        let i = compute_geometry_inertia(&geom, 2.0);
        let r2 = 0.05f64.powi(2);
        let h2 = 0.2f64.powi(2);
        let expected_ixx = 2.0 / 12.0 * (3.0 * r2 + h2);
        let expected_izz = 2.0 / 2.0 * r2;
        assert!((i.ixx - expected_ixx).abs() < 1e-6, "ixx={}", i.ixx);
        assert!((i.iyy - expected_ixx).abs() < 1e-6, "iyy={}", i.iyy);
        assert!((i.izz - expected_izz).abs() < 1e-6, "izz={}", i.izz);
    }

    #[test]
    fn sphere_inertia_values() {
        // 3kg sphere, radius=0.1
        let geom = GeomData::Sphere { radius: 0.1 };
        let i = compute_geometry_inertia(&geom, 3.0);
        let expected = 2.0 / 5.0 * 3.0 * 0.1f64.powi(2);
        assert!((i.ixx - expected).abs() < 1e-6);
        assert!((i.iyy - expected).abs() < 1e-6);
        assert!((i.izz - expected).abs() < 1e-6);
    }

    #[test]
    fn volume_box() {
        let geom = GeomData::Box { hx: 0.5, hy: 0.5, hz: 0.5 };
        let vol = compute_geometry_volume(&geom);
        assert!((vol - 1.0).abs() < 1e-6); // 1x1x1 = 1 m³
    }

    #[test]
    fn volume_cylinder() {
        let geom = GeomData::Cylinder { radius: 1.0, half_length: 0.5 };
        let vol = compute_geometry_volume(&geom);
        assert!((vol - std::f64::consts::PI).abs() < 1e-6); // π×1²×1
    }

    #[test]
    fn volume_sphere() {
        let geom = GeomData::Sphere { radius: 1.0 };
        let vol = compute_geometry_volume(&geom);
        let expected = 4.0 / 3.0 * std::f64::consts::PI;
        assert!((vol - expected).abs() < 1e-6);
    }

    #[test]
    fn combined_inertia_single_centered_visual() {
        // A single visual at origin should produce the same inertia as
        // compute_geometry_inertia, with CoM at origin.
        use nalgebra as na;
        let vis = VisualData {
            origin: na::Isometry3::identity(),
            geometry: GeomData::Box { hx: 0.1, hy: 0.1, hz: 0.1 },
            color: [0.7, 0.7, 0.7, 1.0],
        };
        let result = compute_link_inertia(&[vis], 1.0);
        assert!((result.mass - 1.0).abs() < 1e-15);
        assert!(result.origin.translation.vector.norm() < 1e-10); // CoM at origin
        let expected = compute_geometry_inertia(&GeomData::Box { hx: 0.1, hy: 0.1, hz: 0.1 }, 1.0);
        assert!((result.ixx - expected.ixx).abs() < 1e-10);
        assert!((result.iyy - expected.iyy).abs() < 1e-10);
        assert!((result.izz - expected.izz).abs() < 1e-10);
    }

    #[test]
    fn combined_inertia_offset_visual() {
        // A visual offset along X should shift CoM and increase Iyy, Izz
        // via the parallel axis theorem.
        use nalgebra as na;
        let vis = VisualData {
            origin: na::Isometry3::from_parts(
                na::Translation3::new(1.0, 0.0, 0.0),
                na::UnitQuaternion::identity(),
            ),
            geometry: GeomData::Sphere { radius: 0.1 },
            color: [1.0, 0.0, 0.0, 1.0],
        };
        let result = compute_link_inertia(&[vis], 2.0);
        // CoM should be at (1, 0, 0)
        assert!((result.origin.translation.x - 1.0).abs() < 1e-6);
        assert!(result.origin.translation.y.abs() < 1e-6);
        // With CoM exactly at the sphere center, the inertia should be
        // the same as the sphere inertia (no parallel axis shift).
        let expected = compute_geometry_inertia(&GeomData::Sphere { radius: 0.1 }, 2.0);
        assert!((result.ixx - expected.ixx).abs() < 1e-8);
        assert!((result.iyy - expected.iyy).abs() < 1e-8);
    }

    #[test]
    fn combined_inertia_two_visuals_parallel_axis() {
        // Two identical spheres at ±0.5 on X axis should have:
        // - CoM at origin
        // - Iyy, Izz increased by parallel axis theorem
        use nalgebra as na;
        let vis1 = VisualData {
            origin: na::Isometry3::from_parts(
                na::Translation3::new(0.5, 0.0, 0.0),
                na::UnitQuaternion::identity(),
            ),
            geometry: GeomData::Sphere { radius: 0.1 },
            color: [0.7, 0.7, 0.7, 1.0],
        };
        let vis2 = VisualData {
            origin: na::Isometry3::from_parts(
                na::Translation3::new(-0.5, 0.0, 0.0),
                na::UnitQuaternion::identity(),
            ),
            geometry: GeomData::Sphere { radius: 0.1 },
            color: [0.7, 0.7, 0.7, 1.0],
        };
        let result = compute_link_inertia(&[vis1, vis2], 2.0);
        // CoM at origin
        assert!(result.origin.translation.vector.norm() < 1e-6);
        // Each sphere: mass=1.0, sphere inertia + parallel axis shift d=0.5
        let i_sphere = compute_geometry_inertia(&GeomData::Sphere { radius: 0.1 }, 1.0);
        // Iyy for each = i_sphere.iyy + 1.0 * 0.5² = i_sphere.iyy + 0.25
        let expected_iyy = 2.0 * (i_sphere.iyy + 1.0 * 0.25);
        // Ixx should have NO parallel axis shift (displacement along X, so d_y=d_z=0)
        let expected_ixx = 2.0 * i_sphere.ixx;
        assert!(
            (result.ixx - expected_ixx).abs() < 1e-8,
            "ixx: got {} expected {}",
            result.ixx,
            expected_ixx
        );
        assert!(
            (result.iyy - expected_iyy).abs() < 1e-8,
            "iyy: got {} expected {}",
            result.iyy,
            expected_iyy
        );
    }

    #[test]
    fn inertia_scales_with_mass() {
        let geom = GeomData::Box { hx: 0.1, hy: 0.1, hz: 0.1 };
        let i1 = compute_geometry_inertia(&geom, 1.0);
        let i2 = compute_geometry_inertia(&geom, 5.0);
        assert!((i2.ixx / i1.ixx - 5.0).abs() < 1e-6);
        assert!((i2.iyy / i1.iyy - 5.0).abs() < 1e-6);
        assert!((i2.izz / i1.izz - 5.0).abs() < 1e-6);
    }
}

// ========== Inertia Validation Tests ==========

mod test_inertia_validation {
    use articara::robot::*;

    fn make_link(name: &str, mass: f64, ixx: f64, iyy: f64, izz: f64,
                 ixy: f64, ixz: f64, iyz: f64) -> LinkData {
        LinkData {
            name: name.to_string(),
            visuals: vec![],
            collisions: vec![],
            inertial: InertialData {
                origin: nalgebra::Isometry3::identity(),
                mass,
                ixx, ixy, ixz,
                iyy, iyz, izz,
            },
            collision_enabled: true,
        }
    }

    #[test]
    fn valid_box_inertia_passes() {
        // 1 kg box 0.2×0.2×0.2 m
        let m = 1.0;
        let s = 0.2_f64;
        let i_diag = m / 12.0 * (s * s + s * s); // ~0.006667
        let link = make_link("box", m, i_diag, i_diag, i_diag, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.is_ok(), "Expected OK, got: {:?}", v.issues);
    }

    #[test]
    fn negative_mass_is_error() {
        let link = make_link("bad", -1.0, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_errors());
        assert!(v.issues.iter().any(|i| i.message.contains("negative")));
    }

    #[test]
    fn zero_mass_is_warning() {
        let link = make_link("dummy", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_warnings());
        assert!(!v.has_errors());
    }

    #[test]
    fn negative_diagonal_is_error() {
        let link = make_link("bad_ixx", 1.0, -0.01, 0.01, 0.01, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_errors());
        assert!(v.issues.iter().any(|i| i.message.contains("Ixx") && i.message.contains("negative")));
    }

    #[test]
    fn triangle_inequality_violation() {
        // Izz much larger than Ixx + Iyy
        let link = make_link("bad_tri", 1.0, 0.001, 0.001, 1.0, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_errors());
        assert!(v.issues.iter().any(|i| i.message.contains("Triangle inequality")));
    }

    #[test]
    fn triangle_inequality_satisfied() {
        // Uniform sphere: all diagonal elements equal
        let link = make_link("sphere", 1.0, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(!v.has_errors(), "Unexpected errors: {:?}", v.issues);
    }

    #[test]
    fn not_positive_semi_definite() {
        // Large off-diagonal elements make the matrix non-PSD
        let link = make_link("bad_psd", 1.0, 0.001, 0.001, 0.001, 0.1, 0.1, 0.1);
        let v = validate_inertia(&link);
        assert!(v.has_errors());
        assert!(v.issues.iter().any(|i| i.message.contains("positive semi-definite")));
    }

    #[test]
    fn large_inertia_for_mass_warns() {
        // Mass 1 kg but Ixx = 200 → equivalent radius > 10 m
        let link = make_link("huge", 1.0, 200.0, 200.0, 200.0, 0.0, 0.0, 0.0);
        let v = validate_inertia(&link);
        assert!(v.has_warnings());
        assert!(v.issues.iter().any(|i| i.message.contains("very large")));
    }

    #[test]
    fn validate_all_returns_per_link() {
        let mut model = RobotModel::new_empty("test");
        // new_empty creates "base_link" with mass=1.0 and valid inertia

        // Add a child link
        model.add_child(
            "base_link", "bad_link", "j1",
            "revolute",
            nalgebra::Isometry3::identity(),
            nalgebra::Vector3::z(),
            GeomData::Box { hx: 0.1, hy: 0.1, hz: 0.1 },
            [0.5, 0.5, 0.5, 1.0],
            -1.0, 1.0,
        ).unwrap();
        // Make the child link have bad inertia (negative mass)
        if let Some(idx) = model.link_map.get("bad_link") {
            model.links[*idx].inertial.mass = -1.0;
        }
        let results = validate_all_inertia(&model);
        assert_eq!(results.len(), 2);
        let bad_result = results.iter().find(|r| r.link_name == "bad_link").unwrap();
        assert!(bad_result.has_errors());
        let good_result = results.iter().find(|r| r.link_name == "base_link").unwrap();
        assert!(!good_result.has_errors());
    }

    #[test]
    fn valid_nonzero_offdiag_passes() {
        // A physically valid tensor with small off-diagonal elements
        // that still satisfy PSD
        let link = make_link("tilted", 2.0, 0.05, 0.04, 0.06, 0.001, 0.001, 0.001);
        let v = validate_inertia(&link);
        assert!(!v.has_errors(), "Unexpected errors: {:?}", v.issues);
    }
}

// ============================================================
// Mesh I/O regressions — bugs caught while wiring OBJ support
// and V-HACD save in May 2026. Each test pins one specific
// behaviour that broke at least once; all are self-contained
// (write tiny synthetic URDF + mesh into a tempdir).
// ============================================================
mod test_mesh_io_regressions {
    use super::*;
    use articara::rbd::model::{CollisionData, GeomData};
    use articara::robot::{resolve_package_path, RobotModel};

    /// A 2-triangle Wavefront OBJ (forms a corner of a cube). 4 unique
    /// verts (deduped by tobj's `single_index` pass) → 2 triangles → 12
    /// `f32`s per vert × 3 verts × 2 tris = 36 floats in the flat output.
    const TINY_OBJ: &str = "\
o tri
v 0.0 0.0 0.0
v 1.0 0.0 0.0
v 0.0 1.0 0.0
v 0.0 0.0 1.0
f 1 2 3
f 1 3 4
";

    /// Minimal binary STL with one triangle (header + tri-count + 1 triangle record).
    fn one_tri_stl_bytes() -> Vec<u8> {
        let mut buf = vec![0u8; 80]; // header
        buf.extend_from_slice(&1u32.to_le_bytes()); // 1 triangle
        // normal (0,0,1)
        buf.extend_from_slice(&0f32.to_le_bytes());
        buf.extend_from_slice(&0f32.to_le_bytes());
        buf.extend_from_slice(&1f32.to_le_bytes());
        // 3 vertices in z=0 plane
        for v in [[0f32, 0., 0.], [1., 0., 0.], [0., 1., 0.]] {
            for f in v { buf.extend_from_slice(&f.to_le_bytes()); }
        }
        buf.extend_from_slice(&[0u8, 0u8]); // attribute byte count
        buf
    }

    /// Minimal URDF with one base link carrying a single mesh visual.
    fn tiny_urdf(mesh_uri: &str) -> String {
        format!(
            r#"<?xml version="1.0"?>
<robot name="test_pkg">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_uri}"/>
      </geometry>
    </visual>
  </link>
</robot>
"#
        )
    }

    fn tempdir(prefix: &str) -> PathBuf {
        use std::time::{SystemTime, UNIX_EPOCH};
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos();
        let p = std::env::temp_dir().join(format!("articara_meshio_{prefix}_{nanos}"));
        std::fs::create_dir_all(&p).unwrap();
        p
    }

    /// Lay out a tiny self-contained URDF package at `<root>/test_pkg/`
    /// with `mesh/tri.obj` and a URDF that references it. Returns the
    /// path to the URDF.
    fn write_tiny_obj_package(root: &Path) -> PathBuf {
        let pkg = root.join("test_pkg");
        let mesh_dir = pkg.join("mesh");
        std::fs::create_dir_all(&mesh_dir).unwrap();
        std::fs::write(mesh_dir.join("tri.obj"), TINY_OBJ).unwrap();
        let urdf_path = pkg.join("robot.urdf");
        std::fs::write(
            &urdf_path,
            tiny_urdf("package://test_pkg/mesh/tri.obj"),
        )
        .unwrap();
        urdf_path
    }

    // ── Lesson 1: convert_geometry must dispatch on extension ────────────
    /// Regression: a URDF that references a `.obj` file must populate
    /// `GeomData::Mesh.vertices` (broken when `convert_geometry`
    /// unconditionally called the STL parser).
    #[test]
    fn urdf_with_obj_mesh_loads_vertices() {
        let dir = tempdir("urdf_obj");
        let urdf_path = write_tiny_obj_package(&dir);
        let model = RobotModel::from_urdf(&urdf_path).expect("load URDF");

        let mut obj_meshes = 0;
        for link in &model.links {
            for v in &link.visuals {
                if let GeomData::Mesh { mesh, filename, .. } = &v.geometry {
                    let vertices = mesh.to_flat_vertices_f32();
                    if filename.as_deref().map(|s| s.ends_with(".obj")).unwrap_or(false) {
                        assert!(
                            !vertices.is_empty(),
                            "OBJ vertices empty — convert_geometry probably regressed to STL-only"
                        );
                        // 2 tris × 3 verts × 6 floats = 36 floats expected
                        assert_eq!(vertices.len(), 36, "expected 2 tris worth of floats");
                        obj_meshes += 1;
                    }
                }
            }
        }
        assert_eq!(obj_meshes, 1, "expected exactly one OBJ visual");
        std::fs::remove_dir_all(&dir).ok();
    }

    // ── Lesson 2: resolve_package_path must handle direct-in-package layout ──
    /// Regression: when the URDF lives directly inside its package dir
    /// (`<pkg>/foo.urdf` rather than the ROS-canonical `<pkg>/urdf/foo.urdf`),
    /// `package://<pkg>/mesh/x.stl` must still resolve. The fix added a
    /// second-candidate lookup (`package_dir.join(pkg).join(rel)`) on top
    /// of the original ROS-only `package_dir.join(rel)`.
    #[test]
    fn resolve_package_path_direct_in_package_layout() {
        let dir = tempdir("direct_layout");
        let pkg = dir.join("my_pkg");
        std::fs::create_dir_all(pkg.join("mesh")).unwrap();
        let mesh_path = pkg.join("mesh/foo.stl");
        std::fs::write(&mesh_path, b"").unwrap();

        // Direct layout: URDF is at <root>/my_pkg/foo.urdf, so the loader
        // computes `package_dir = <root>` (parent of urdf_dir = my_pkg).
        let resolved = resolve_package_path(
            "package://my_pkg/mesh/foo.stl",
            &dir, // = package_dir
        );
        assert_eq!(
            resolved, mesh_path,
            "resolver should fall back to package_dir.join(pkg_name).join(rel)"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    // ── Lesson 3: V-HACD output survives save_urdf ───────────────────────
    /// Regression: a `GeomData::Mesh` with `filename: None` (the shape
    /// V-HACD produces) must be materialised to a real STL file at save
    /// time. Pre-fix, `geom_to_urdf_geom` wrote `filename="mesh.stl"`
    /// and silently dropped the vertex data.
    #[test]
    fn vhacd_decomposed_mesh_survives_urdf_save() {
        let dir = tempdir("vhacd_urdf");
        let urdf_path = write_tiny_obj_package(&dir);
        let mut model = RobotModel::from_urdf(&urdf_path).expect("load URDF");

        // Inject a fake V-HACD output as an additional collision on link 0.
        let fake_verts = one_tri_flat_verts();
        model.links[0].collisions.push(CollisionData {
            origin: nalgebra::Isometry3::identity(),
            geometry: GeomData::Mesh {
                mesh: std::sync::Arc::new(
                    misarta::mesh::MeshData::from_flat_vertices_f32(&fake_verts),
                ),
                filename: None,
                scale: None,
            },
            physics: None,
        });
        let link_name = model.links[0].name.clone();
        let col_idx = model.links[0].collisions.len() - 1;

        model.save_urdf().expect("save_urdf");

        // STL must exist next to the URDF — file path is deterministic.
        let expected_stl = urdf_path
            .parent()
            .unwrap()
            .join(format!("meshes/decomposed/{link_name}_col_{col_idx}.stl"));
        assert!(
            expected_stl.exists(),
            "decomposed STL not materialised at {expected_stl:?}"
        );

        // Reload and confirm the round-tripped mesh has vertices populated.
        let reloaded = RobotModel::from_urdf(&urdf_path).expect("reload URDF");
        let target_link = reloaded
            .links
            .iter()
            .find(|l| l.name == link_name)
            .expect("link survives round-trip");
        let any_decomposed_loaded = target_link.collisions.iter().any(|c| {
            matches!(
                &c.geometry,
                GeomData::Mesh { filename: Some(f), mesh, .. }
                    if f.contains("meshes/decomposed") && mesh.num_triangles() > 0
            )
        });
        assert!(
            any_decomposed_loaded,
            "reloaded URDF should reference the materialised STL with non-empty verts"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    // ── Lesson 4: V-HACD output survives save_as_misa ────────────────────
    /// Regression: same as #3 but for the `.misa` save path
    /// (`materialize_decomposed_meshes` is called from there too).
    #[test]
    fn vhacd_decomposed_mesh_survives_misa_save() {
        let dir = tempdir("vhacd_misa");
        let urdf_path = write_tiny_obj_package(&dir);
        let mut model = RobotModel::from_urdf(&urdf_path).expect("load URDF");
        model.links[0].collisions.push(CollisionData {
            origin: nalgebra::Isometry3::identity(),
            geometry: GeomData::Mesh {
                mesh: std::sync::Arc::new(
                    misarta::mesh::MeshData::from_flat_vertices_f32(&one_tri_flat_verts()),
                ),
                filename: None,
                scale: None,
            },
            physics: None,
        });
        let link_name = model.links[0].name.clone();
        let col_idx = model.links[0].collisions.len() - 1;
        let misa_path = dir.join("out.misa");

        model.save_as_misa(&misa_path).expect("save_as_misa");

        let expected_stl = dir.join(format!(
            "meshes/decomposed/{link_name}_col_{col_idx}.stl"
        ));
        assert!(
            expected_stl.exists(),
            "decomposed STL not materialised at {expected_stl:?}"
        );
        // save_as_misa works on a clone — caller's model must stay untouched.
        assert!(
            matches!(
                &model.links[0].collisions[col_idx].geometry,
                GeomData::Mesh { filename: None, .. }
            ),
            "save_as_misa must not mutate caller's model"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    // ── Lesson 5: save_as_misa copies referenced meshes ──────────────────
    /// Regression: `save_as_misa` to a fresh directory must copy every
    /// referenced mesh next to the `.misa`, so the `AssetSource` sandbox
    /// can resolve them on reload. Pre-fix, the OBJ files were left at
    /// the original URDF location and the reload reported them as
    /// `missing_meshes`.
    #[test]
    fn save_as_misa_copies_referenced_meshes_to_output_dir() {
        let src_dir = tempdir("misa_copy_src");
        let urdf_path = write_tiny_obj_package(&src_dir);
        let model = RobotModel::from_urdf(&urdf_path).expect("load URDF");

        let dst_dir = tempdir("misa_copy_dst");
        let misa_path = dst_dir.join("out.misa");
        model.save_as_misa(&misa_path).expect("save_as_misa");

        // .misa references `mesh/tri.obj` (relative). That file must now
        // exist relative to the .misa.
        let expected = dst_dir.join("mesh/tri.obj");
        assert!(
            expected.exists(),
            "referenced OBJ was not copied to {expected:?}"
        );
        std::fs::remove_dir_all(&src_dir).ok();
        std::fs::remove_dir_all(&dst_dir).ok();
    }

    // ── Lesson 6: .misa loader dispatches on extension (OBJ via .misa) ───
    /// Regression: end-to-end. URDF references an OBJ → save as `.misa` →
    /// reload via `from_misa_with_report`. Must report no missing meshes
    /// and the in-memory model's OBJ visual must have populated vertices.
    /// This catches both the misarta `parse_mesh_bytes` dispatch AND the
    /// articara `convert_misa_geom` dispatch (both were STL-only).
    #[test]
    fn misa_roundtrip_loads_obj() {
        let src_dir = tempdir("misa_obj_rt_src");
        let urdf_path = write_tiny_obj_package(&src_dir);
        let model = RobotModel::from_urdf(&urdf_path).expect("load URDF");

        let dst_dir = tempdir("misa_obj_rt_dst");
        let misa_path = dst_dir.join("out.misa");
        model.save_as_misa(&misa_path).expect("save_as_misa");

        let (loaded, report) =
            RobotModel::from_misa_with_report(&misa_path).expect("reload misa");
        assert!(
            report.missing_meshes.is_empty(),
            "expected no missing meshes after .misa round-trip, got: {:?}",
            report.missing_meshes
        );

        let mut obj_meshes_with_verts = 0;
        for link in &loaded.links {
            for v in &link.visuals {
                if let GeomData::Mesh { filename: Some(f), mesh, .. } = &v.geometry {
                    if f.to_lowercase().ends_with(".obj") && mesh.num_triangles() > 0 {
                        obj_meshes_with_verts += 1;
                    }
                }
            }
        }
        assert!(
            obj_meshes_with_verts > 0,
            "expected at least one OBJ visual to have vertices after .misa reload"
        );
        std::fs::remove_dir_all(&src_dir).ok();
        std::fs::remove_dir_all(&dst_dir).ok();
    }

    /// Build a single-triangle flat `[x,y,z,nx,ny,nz]` vertex buffer
    /// (matching what `load_stl_mesh` / `load_obj_mesh` emit).
    fn one_tri_flat_verts() -> Vec<f32> {
        vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
        ]
    }

    // Unused at present but kept available if a future test wants a
    // genuine STL file to round-trip.
    #[allow(dead_code)]
    fn write_tiny_stl(path: &Path) {
        std::fs::write(path, one_tri_stl_bytes()).unwrap();
    }
}
