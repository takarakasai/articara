//! Collision detection helpers.
//!
//! Delegates to `misarta::collision` for the actual collision queries,
//! building a `GeometryModel` from the `RobotModel`'s collision geometry.

use crate::robot::RobotModel;

#[derive(Debug, Clone)]
pub struct CollisionHit {
    pub link_a_idx: usize,
    pub collision_a_idx: usize,
    pub link_b_idx: usize,
    pub collision_b_idx: usize,
}

pub fn self_collision_hits(robot: &RobotModel, ignore_adjacent_links: bool) -> Vec<CollisionHit> {
    let mc = robot.mc();
    let q = mc.build_q(robot);
    let (gmodel, geo_map) = robot.build_collision_geometry_with_map();

    let pairs = misarta::collision::collision_pairs(
        &mc.model,
        &gmodel,
        &q,
        ignore_adjacent_links,
    );

    pairs
        .iter()
        .filter_map(|p| {
            let (la, ca) = geo_map.get(p.a)?;
            let (lb, cb) = geo_map.get(p.b)?;
            Some(CollisionHit {
                link_a_idx: *la,
                collision_a_idx: *ca,
                link_b_idx: *lb,
                collision_b_idx: *cb,
            })
        })
        .collect()
}

pub fn has_self_collision(robot: &RobotModel, ignore_adjacent_links: bool) -> bool {
    let mc = robot.mc();
    let q = mc.build_q(robot);
    let (gmodel, _) = robot.build_collision_geometry_with_map();

    misarta::collision::has_collision(&mc.model, &gmodel, &q, ignore_adjacent_links)
}

pub fn minimum_separation_distance(
    robot: &RobotModel,
    ignore_adjacent_links: bool,
) -> Option<f32> {
    let mc = robot.mc();
    let q = mc.build_q(robot);
    let (gmodel, _) = robot.build_collision_geometry_with_map();

    misarta::collision::minimum_distance(&mc.model, &gmodel, &q, ignore_adjacent_links)
        .map(|d| d as f32)
}

impl RobotModel {
    pub fn has_self_collision(&self, ignore_adjacent_links: bool) -> bool {
        has_self_collision(self, ignore_adjacent_links)
    }

    pub fn self_collision_hits(&self, ignore_adjacent_links: bool) -> Vec<CollisionHit> {
        self_collision_hits(self, ignore_adjacent_links)
    }

    pub fn minimum_separation_distance(&self, ignore_adjacent_links: bool) -> Option<f32> {
        minimum_separation_distance(self, ignore_adjacent_links)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::{CollisionData, GeomData, InertialData, JointData, LinkData};
    use nalgebra as na;
    use std::collections::HashMap;

    fn sphere_collision(radius: f32) -> CollisionData {
        CollisionData {
            origin: na::Isometry3::identity(),
            geometry: GeomData::Sphere { radius },
        }
    }

    fn inertial() -> InertialData {
        InertialData {
            origin: na::Isometry3::identity(),
            mass: 1.0,
            ixx: 1.0,
            ixy: 0.0,
            ixz: 0.0,
            iyy: 1.0,
            iyz: 0.0,
            izz: 1.0,
        }
    }

    fn two_link_model(offset_x: f32) -> RobotModel {
        let links = vec![
            LinkData {
                name: "base".to_string(),
                visuals: vec![],
                collisions: vec![sphere_collision(0.5)],
                inertial: inertial(),
            },
            LinkData {
                name: "link1".to_string(),
                visuals: vec![],
                collisions: vec![sphere_collision(0.5)],
                inertial: inertial(),
            },
        ];

        let joints = vec![JointData {
            name: "joint0".to_string(),
            joint_type: "fixed".to_string(),
            parent_link: "base".to_string(),
            child_link: "link1".to_string(),
            origin: na::Isometry3::new(
                na::Vector3::new(offset_x, 0.0, 0.0),
                na::Vector3::zeros(),
            ),
            axis: na::Vector3::z_axis().into_inner(),
            lower: 0.0,
            upper: 0.0,
            effort: 0.0,
            velocity: 0.0,
            actuator_mode: crate::rbd::model::ActuatorMode::default(),
            actuator_kp: 50.0,
            actuator_kv: 5.0,
        }];

        let mut link_map = HashMap::new();
        link_map.insert("base".to_string(), 0);
        link_map.insert("link1".to_string(), 1);

        let mut joint_map = HashMap::new();
        joint_map.insert("joint0".to_string(), 0);

        let mut children_joints = HashMap::new();
        children_joints.insert("base".to_string(), vec![0]);

        let mut model = RobotModel {
            name: "test".to_string(),
            links,
            joints,
            link_map,
            joint_map,
            root_link: "base".to_string(),
            children_joints,
            materials: HashMap::new(),
            joint_positions: vec![0.0],
            source_path: None,
            base_transform: na::Isometry3::identity(),
            misarta_cache: None,
            loop_closures: Vec::new(),
            poses: Vec::new(),
            collision_pairs: Vec::new(),
        };
        model.rebuild_misarta_model();
        model
    }

    #[test]
    fn self_collision_detected_for_overlapping_spheres() {
        let model = two_link_model(0.6);
        assert!(has_self_collision(&model, false));
        assert!(!self_collision_hits(&model, false).is_empty());
    }

    #[test]
    fn adjacent_pair_can_be_ignored() {
        let model = two_link_model(0.6);
        assert!(!has_self_collision(&model, true));
    }

    #[test]
    fn self_collision_not_detected_when_separated() {
        let model = two_link_model(2.0);
        assert!(!has_self_collision(&model, false));
    }

    #[test]
    fn minimum_distance_for_two_spheres() {
        let model = two_link_model(2.0);
        let d = minimum_separation_distance(&model, false).unwrap();
        assert!((d - 1.0).abs() < 1e-5);
    }
}
