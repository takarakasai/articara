//! Isaac Sim export — generates a Python script that creates the robot in USD.
//!
//! Isaac Sim natively imports URDF via `omni.isaac.urdf`. We also generate
//! a standalone Python script that can be run in Isaac Sim's script editor
//! to create the robot programmatically.

use std::path::Path;

use crate::robot::*;

/// Export a Python script for Isaac Sim that loads the robot from URDF.
///
/// The script uses `omni.isaac.urdf` extension to import the URDF,
/// and also configures drive properties for interactive joints.
pub fn export_isaac_python(model: &RobotModel, urdf_relative_path: &str) -> String {
    let mut s = String::new();

    s.push_str("# Auto-generated Isaac Sim import script\n");
    s.push_str("# Run this in Isaac Sim's Script Editor or via command line.\n");
    s.push_str("#\n");
    s.push_str(&format!("# Robot: {}\n", model.name));
    s.push_str(&format!("# Links: {}, Joints: {}\n", model.links.len(), model.joints.len()));
    s.push_str("#\n");
    s.push_str("# Usage:\n");
    s.push_str("#   1. Open Isaac Sim\n");
    s.push_str("#   2. Window -> Script Editor\n");
    s.push_str("#   3. Paste and run this script\n");
    s.push_str("#   - OR run: ~/.local/share/ov/pkg/isaac-sim-*/python.sh this_script.py\n\n");

    s.push_str("import omni\n");
    s.push_str("from omni.isaac.urdf import _urdf\n");
    s.push_str("from pxr import UsdPhysics, Sdf, Gf, UsdGeom\n\n");

    // URDF import config
    s.push_str("# === URDF Import Configuration ===\n");
    s.push_str(&format!("URDF_PATH = r\"{}\"\n\n", urdf_relative_path));

    s.push_str("urdf_interface = _urdf.acquire_urdf_interface()\n\n");

    s.push_str("import_config = _urdf.ImportConfig()\n");
    s.push_str("import_config.merge_fixed_joints = False\n");
    s.push_str("import_config.fix_base = True\n");
    s.push_str("import_config.import_inertia_tensor = True\n");
    s.push_str("import_config.distance_scale = 1.0\n");
    s.push_str("import_config.density = 0.0  # Use URDF mass values\n");
    s.push_str("import_config.default_drive_type = 1  # Position drive\n");
    s.push_str("import_config.default_drive_strength = 1e4\n");
    s.push_str("import_config.default_position_drive_damping = 1e3\n");
    s.push_str("import_config.convex_decomp = False\n");
    s.push_str("import_config.self_collision = False\n\n");

    s.push_str("# Parse and import\n");
    s.push_str("parsed_result = urdf_interface.parse_urdf(URDF_PATH, import_config)\n");
    s.push_str(&format!(
        "robot_path = urdf_interface.import_robot(\n    URDF_PATH,\n    parsed_result,\n    import_config,\n    \"/World/{}\"\n)\n\n",
        model.name
    ));

    // Joint drive configuration
    s.push_str("# === Joint Drive Configuration ===\n");
    s.push_str("stage = omni.usd.get_context().get_stage()\n\n");

    for joint in &model.joints {
        if joint.joint_type == "fixed" {
            continue;
        }
        s.push_str(&format!(
            "# Joint: {} ({})\n",
            joint.name, joint.joint_type
        ));
        s.push_str(&format!(
            "joint_prim = stage.GetPrimAtPath(\"/World/{}/{}\")\n",
            model.name, joint.name
        ));
        s.push_str("if joint_prim.IsValid():\n");
        if joint.joint_type == "revolute" || joint.joint_type == "continuous" {
            s.push_str("    drive = UsdPhysics.DriveAPI.Apply(joint_prim, \"angular\")\n");
            s.push_str("    drive.CreateTypeAttr(\"force\")\n");
            s.push_str(&format!(
                "    drive.CreateMaxForceAttr({})\n",
                joint.effort
            ));
            s.push_str("    drive.CreateDampingAttr(1e3)\n");
            s.push_str("    drive.CreateStiffnessAttr(1e4)\n");
        } else if joint.joint_type == "prismatic" {
            s.push_str("    drive = UsdPhysics.DriveAPI.Apply(joint_prim, \"linear\")\n");
            s.push_str("    drive.CreateTypeAttr(\"force\")\n");
            s.push_str(&format!(
                "    drive.CreateMaxForceAttr({})\n",
                joint.effort
            ));
            s.push_str("    drive.CreateDampingAttr(1e3)\n");
            s.push_str("    drive.CreateStiffnessAttr(1e4)\n");
        }
        s.push_str("\n");
    }

    // Physics scene setup
    s.push_str("# === Physics Scene ===\n");
    s.push_str("physics_scene_path = \"/World/PhysicsScene\"\n");
    s.push_str("if not stage.GetPrimAtPath(physics_scene_path).IsValid():\n");
    s.push_str("    UsdPhysics.Scene.Define(stage, physics_scene_path)\n");
    s.push_str("    physics_prim = stage.GetPrimAtPath(physics_scene_path)\n");
    s.push_str("    physics_prim.CreateAttribute(\"physics:gravityDirection\", Sdf.ValueTypeNames.Vector3f).Set(Gf.Vec3f(0, 0, -1))\n");
    s.push_str("    physics_prim.CreateAttribute(\"physics:gravityMagnitude\", Sdf.ValueTypeNames.Float).Set(9.81)\n\n");

    // Ground plane
    s.push_str("# === Ground Plane ===\n");
    s.push_str("ground_path = \"/World/GroundPlane\"\n");
    s.push_str("if not stage.GetPrimAtPath(ground_path).IsValid():\n");
    s.push_str("    plane = UsdGeom.Plane.Define(stage, ground_path)\n");
    s.push_str("    plane.CreateAxisAttr(\"Z\")\n");
    s.push_str("    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(ground_path))\n\n");

    s.push_str(&format!(
        "print(\"Robot '{}' imported successfully.\")\n",
        model.name
    ));

    s
}

/// Export Isaac-compatible files: a URDF + Python import script.
pub fn export_isaac_to_dir(
    model: &RobotModel,
    output_dir: &Path,
) -> Result<(), String> {
    // First export the URDF (using existing URDF export)
    let urdf_subdir = output_dir.join("urdf");
    std::fs::create_dir_all(&urdf_subdir)
        .map_err(|e| format!("Create urdf dir: {e}"))?;

    let urdf_filename = format!("{}.urdf", model.name);
    let urdf_path = urdf_subdir.join(&urdf_filename);
    model.export_urdf_to_file(&urdf_path)?;

    // Generate Python script
    let relative_urdf = format!("urdf/{urdf_filename}");
    let script = export_isaac_python(model, &relative_urdf);
    let script_path = output_dir.join(format!("import_{}.py", model.name));
    std::fs::write(&script_path, &script)
        .map_err(|e| format!("Write Isaac script: {e}"))?;

    log::info!(
        "Isaac export: URDF at {:?}, script at {:?}",
        urdf_path,
        script_path
    );
    Ok(())
}
