//! Go2 MJCF を読み込んで `.misa` を書き出す。手動再現用に、4 脚すべてを
//! 同じ角度で屈曲した姿勢を `bent_home` として Pose に登録しておく。
//!
//! 実行:
//!   cargo run --no-default-features --example go2_export_misa
//!
//! 出力: models/unitree_go2/go2.misa

use articara::mjcf;
use articara::rbd::model::{CollisionData, MjcfPhysics, NamedPose};
use articara::robot::*;
use misarta::trajectory::InterpolationKind;
use nalgebra as na;
use std::collections::BTreeMap;

fn main() {
    let src = std::path::Path::new("models/unitree_go2/go2.xml");
    let mut model = mjcf::import_mjcf(src).expect("Load Go2 MJCF");

    // 合成 foot リンク (FL/FR/RL/RR_foot) を calf に fixed で生やす。Menagerie
    // の Go2 は足が calf 内の collision sphere geom として表現されていて
    // 独立した body ではないが、IK ターゲットと collision を foot リンクに
    // 集約したいので:
    //   1. 各 foot リンクに visual + collision の sphere (r=0.022) を入れる
    //   2. 親 calf の collision 配列から重複する foot sphere (= 同じ offset
    //      & 同じ半径の sphere) を削除する。両方残すと地面接触点が二重に
    //      立って solver が不安定化する。
    let foot_radius = 0.022_f32;
    let foot_offset = na::Vector3::new(-0.002_f32, 0.0, -0.213);
    for (foot, calf) in [
        ("FL_foot", "FL_calf"),
        ("FR_foot", "FR_calf"),
        ("RL_foot", "RL_calf"),
        ("RR_foot", "RR_calf"),
    ] {
        let origin = na::Isometry3::from_parts(
            na::Translation3::from(foot_offset),
            na::UnitQuaternion::identity(),
        );
        // 1. Visual + joint を生やす (add_child は visuals だけ埋める)。
        let (foot_li, _) = model
            .add_child(
                calf, foot, &format!("{foot}_fixed"), "fixed", origin,
                na::Vector3::z(),
                GeomData::Sphere { radius: foot_radius },
                [0.5, 0.5, 0.5, 1.0],
                0.0, 0.0,
            )
            .unwrap();
        // 2. 同じ sphere を collisions にも追加 (origin = link 原点)。
        //    Go2 MJCF の class="foot" と同じ物理パラメタを焼き込んでおく:
        //      friction = "0.8 0.02 0.01"  (tangential μ + torsional + rolling)
        //      condim   = 6                (full friction including rolling)
        //      priority = 1                (geom-vs-geom contact resolution)
        //      solimp   = "0.015 1 0.022"  (soft-contact impedance curve)
        //    これらが無いと既定 (μ=0.6 / condim=3) で sphere が地面を転がり、
        //    forward grip が半減して trunk が後方に滑る → 起動時の発散も起こる。
        model.links[foot_li].collisions.push(CollisionData {
            origin: na::Isometry3::identity(),
            geometry: GeomData::Sphere { radius: foot_radius },
            physics: Some(MjcfPhysics {
                friction: Some([0.8, 0.02, 0.01]),
                condim: Some(6),
                priority: Some(1),
                solimp: Some([0.015, 1.0, 0.022]),
                margin: None,
            }),
        });
        // 3. calf 側の重複 foot collision sphere を削除。Go2 MJCF の
        //    class="foot" は半径 0.022 + 足首付近 (z ≈ -0.213) の sphere。
        //    calf の他の collision (太腿付近の box / cylinder, z > -0.2) と
        //    は z 位置で簡単に区別できる。
        // Go2 MJCF はインポート時に
        //   - class="foot" の sphere (r=0.022) — class-inherited pos が
        //     articara::mjcf::import に反映されないため origin (0,0,0) で
        //     入っていることがある。半径だけで一意に識別。
        //   - class="visual" の mesh (calf_0/calf_1/foot.obj) — articara
        //     の importer が `contype=0` 指定を無視するため collision に
        //     も重複登録される。
        // どちらも foot リンクの sphere collision で置き換えるので calf
        // からは mesh 型 + 小 sphere をまとめて除去。残るのは cylinder
        // (太腿付近の box/cylinder 近似) のみで、これらが正規の calf
        // collision。
        let calf_li = model.link_map[calf];
        model.links[calf_li].collisions.retain(|c| match &c.geometry {
            GeomData::Sphere { radius } => (radius - foot_radius).abs() >= 1e-4,
            GeomData::Mesh { .. } => false,
            _ => true,
        });
    }
    model.rebuild_misarta_model();
    // NOTE: foot link collision は球 1 点だけになるので、Go2 MJCF が元々
    // `class="foot"` で持っていた condim=6 + 高摩擦 (mu=0.8) + 転がり摩擦が
    // 失われ、本 .misa を MuJoCo に流すと既定 Coulomb 摩擦のみで地面を蹴る
    // ことになる。歩行距離が原 MJCF を直接使ったときの ~半分になる点に
    // 注意 (forward grip 不足)。CollisionData に friction / condim を持た
    // せれば解消できるが現状は未対応。

    // Go2 home keyframe (hip=0, thigh=0.9, calf=-1.8) を「全脚同角度の屈曲姿勢」
    // としてそのまま Pose にする。
    let bent: [(&str, f64); 12] = [
        ("FL_hip_joint", 0.0), ("FL_thigh_joint", 0.9), ("FL_calf_joint", -1.8),
        ("FR_hip_joint", 0.0), ("FR_thigh_joint", 0.9), ("FR_calf_joint", -1.8),
        ("RL_hip_joint", 0.0), ("RL_thigh_joint", 0.9), ("RL_calf_joint", -1.8),
        ("RR_hip_joint", 0.0), ("RR_thigh_joint", 0.9), ("RR_calf_joint", -1.8),
    ];

    // Pose 適用前に model.joint_positions を bent に揃えておく (.misa の
    // current pose としても保存されるよう)。
    for (name, q) in bent.iter() {
        let ji = *model.joint_map.get(*name)
            .unwrap_or_else(|| panic!("joint not found: {name}"));
        model.joint_positions[ji] = *q;
    }
    model.rebuild_misarta_model();

    let mut angles = BTreeMap::new();
    for (name, q) in bent.iter() {
        angles.insert((*name).to_string(), *q);
    }
    model.poses.push(NamedPose {
        name: "bent_home".into(),
        angles,
        duration: 1.5,
        kind: InterpolationKind::QuinticSmooth,
    });

    let out = std::path::Path::new("models/unitree_go2/go2.misa");
    model.save_as_misa(out).expect("save_as_misa");
    println!("Wrote: {}", out.display());
    println!("  joints: {} (movable: {})",
        model.joints.len(),
        model.joints.iter().filter(|j| j.joint_type != "fixed").count(),
    );
    println!("  links:  {}", model.links.len());
    println!("  poses:  {} (latest='{}')",
        model.poses.len(),
        model.poses.last().map(|p| p.name.as_str()).unwrap_or(""),
    );
}
