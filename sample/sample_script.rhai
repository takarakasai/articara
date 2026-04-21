// sample_script.rhai — Articara scripting demo
//
// Usage:
//   cargo run --features scripting -- --script sample/sample_script.rhai
//
// Or pre-load a model:
//   cargo run --features scripting -- --script sample/sample_script.rhai tests/fixtures/urdf/test_robot.urdf

// ── 1. Load a model ──
if !has_model() {
    load("tests/fixtures/urdf/test_robot.urdf");
}

print("Model: " + model_name());
print("Links: " + num_links());
print("Joints: " + num_joints());

// ── 2. Query link/joint info ──
let names = joint_names();
print("Joint names: " + names);

for jn in names {
    let lim = joint_limits(jn);
    print("  " + jn + ": type=" + joint_type(jn)
          + "  pos=" + joint_pos(jn)
          + "  limits=[" + lim[0] + ", " + lim[1] + "]");
}

// ── 3. Set joint positions ──
if names.len() >= 2 {
    set_joint(names[0], 0.5);
    set_joint(names[1], -0.3);
    print("After set_joint:");
    print("  " + names[0] + " = " + joint_pos(names[0]));
    print("  " + names[1] + " = " + joint_pos(names[1]));
}

// ── 4. Compute FK ──
let links = link_names();
print("\nFK results:");
for ln in links {
    let p = link_pos(ln);
    if p.len() == 3 {
        print("  " + ln + ": [" + p[0] + ", " + p[1] + ", " + p[2] + "]");
    }
}

// ── 5. Run IK ──
let ee = links[links.len() - 1];
let p0 = link_pos(ee);
print("\nIK target link: " + ee);
print("  Before IK: [" + p0[0] + ", " + p0[1] + ", " + p0[2] + "]");

let tx = p0[0] + 0.02;
let ty = p0[1];
let tz = p0[2] + 0.02;
let err = ik_steps(ee, tx, ty, tz, 50);
let p1 = link_pos(ee);
print("  After IK:  [" + p1[0] + ", " + p1[1] + ", " + p1[2] + "]");
print("  Target:    [" + tx + ", " + ty + ", " + tz + "]");
print("  Error: " + err);

// ── 6. Math demo ──
print("\nMath: sin(PI()/2) = " + sin(PI() / 2.0));
print("Math: to_deg(1.0) = " + to_deg(1.0));

print("\n✔ Script completed successfully");
