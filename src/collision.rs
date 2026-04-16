//! Collision detection helpers based on `parry3d`.
//!
//! This module provides self-collision checks for `RobotModel` using each
//! link's collision geometry and current joint configuration.

use crate::robot::{GeomData, RobotModel};
use nalgebra as na;
use parry3d::query;
use parry3d::shape::{Ball, Cuboid, Cylinder, SharedShape, TriMesh};
use std::collections::{HashMap, HashSet};

#[derive(Debug, Clone)]
pub struct CollisionObject {
    pub link_idx: usize,
    pub collision_idx: usize,
    pub link_name: String,
    pub world_pose: na::Isometry3<f32>,
    pub shape: SharedShape,
}

#[derive(Debug, Clone)]
pub struct CollisionHit {
    pub link_a_idx: usize,
    pub collision_a_idx: usize,
    pub link_b_idx: usize,
    pub collision_b_idx: usize,
}

fn mesh_to_trimesh(vertices: &[f32], scale: Option<[f32; 3]>) -> Option<TriMesh> {
    if vertices.len() < 18 {
        return None;
    }
    let s = scale.unwrap_or([1.0, 1.0, 1.0]);

    let n_verts = vertices.len() / 6;
    if n_verts < 3 {
        return None;
    }

    let mut points = Vec::with_capacity(n_verts);
    for i in 0..n_verts {
        let base = i * 6;
        points.push(na::Point3::new(
            vertices[base] * s[0],
            vertices[base + 1] * s[1],
            vertices[base + 2] * s[2],
        ));
    }

    let mut indices = Vec::new();
    for i in (0..n_verts).step_by(3) {
        if i + 2 < n_verts {
            indices.push([i as u32, (i + 1) as u32, (i + 2) as u32]);
        }
    }
    if indices.is_empty() {
        return None;
    }

    TriMesh::new(points, indices).ok()
}

fn collision_shape_and_local_pose(geom: &GeomData) -> Option<(SharedShape, na::Isometry3<f32>)> {
    match geom {
        GeomData::Box { hx, hy, hz } => Some((
            SharedShape::new(Cuboid::new(na::Vector3::new(*hx, *hy, *hz))),
            na::Isometry3::identity(),
        )),
        GeomData::Sphere { radius } => {
            Some((SharedShape::new(Ball::new(*radius)), na::Isometry3::identity()))
        }
        GeomData::Cylinder {
            radius,
            half_length,
        } => {
            let shape = SharedShape::new(Cylinder::new(*half_length, *radius));
            let rot = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), std::f32::consts::FRAC_PI_2);
            Some((shape, na::Isometry3::from_parts(na::Translation3::identity(), rot)))
        }
        GeomData::Mesh {
            vertices,
            scale,
            ..
        } => {
            let trimesh = mesh_to_trimesh(vertices, *scale)?;
            Some((SharedShape::new(trimesh), na::Isometry3::identity()))
        }
    }
}

fn ignored_adjacent_link_pairs(robot: &RobotModel) -> HashSet<(usize, usize)> {
    let mut ignored = HashSet::new();
    for joint in &robot.joints {
        let Some(&p) = robot.link_map.get(&joint.parent_link) else {
            continue;
        };
        let Some(&c) = robot.link_map.get(&joint.child_link) else {
            continue;
        };
        let pair = if p < c { (p, c) } else { (c, p) };
        ignored.insert(pair);
    }
    ignored
}

pub fn build_collision_objects(
    robot: &RobotModel,
    transforms: &HashMap<String, na::Isometry3<f32>>,
) -> Vec<CollisionObject> {
    let mut out = Vec::new();

    for (li, link) in robot.links.iter().enumerate() {
        let link_tf = transforms
            .get(&link.name)
            .copied()
            .unwrap_or(na::Isometry3::identity());

        for (ci, col) in link.collisions.iter().enumerate() {
            if let Some((shape, local_shape_pose)) = collision_shape_and_local_pose(&col.geometry) {
                let world_pose = link_tf * col.origin * local_shape_pose;
                out.push(CollisionObject {
                    link_idx: li,
                    collision_idx: ci,
                    link_name: link.name.clone(),
                    world_pose,
                    shape,
                });
            }
        }
    }

    out
}

pub fn self_collision_hits(robot: &RobotModel, ignore_adjacent_links: bool) -> Vec<CollisionHit> {
    let transforms = robot.compute_transforms();
    let objects = build_collision_objects(robot, &transforms);
    let ignored_pairs = if ignore_adjacent_links {
        ignored_adjacent_link_pairs(robot)
    } else {
        HashSet::new()
    };

    let mut hits = Vec::new();
    for i in 0..objects.len() {
        for j in (i + 1)..objects.len() {
            let a = &objects[i];
            let b = &objects[j];

            if a.link_idx == b.link_idx {
                continue;
            }

            let pair = if a.link_idx < b.link_idx {
                (a.link_idx, b.link_idx)
            } else {
                (b.link_idx, a.link_idx)
            };

            if ignored_pairs.contains(&pair) {
                continue;
            }

            if query::intersection_test(&a.world_pose, &*a.shape, &b.world_pose, &*b.shape)
                .unwrap_or(false)
            {
                hits.push(CollisionHit {
                    link_a_idx: a.link_idx,
                    collision_a_idx: a.collision_idx,
                    link_b_idx: b.link_idx,
                    collision_b_idx: b.collision_idx,
                });
            }
        }
    }
    hits
}

pub fn has_self_collision(robot: &RobotModel, ignore_adjacent_links: bool) -> bool {
    !self_collision_hits(robot, ignore_adjacent_links).is_empty()
}

pub fn minimum_separation_distance(
    robot: &RobotModel,
    ignore_adjacent_links: bool,
) -> Option<f32> {
    let transforms = robot.compute_transforms();
    let objects = build_collision_objects(robot, &transforms);
    let ignored_pairs = if ignore_adjacent_links {
        ignored_adjacent_link_pairs(robot)
    } else {
        HashSet::new()
    };

    let mut min_dist: Option<f32> = None;
    for i in 0..objects.len() {
        for j in (i + 1)..objects.len() {
            let a = &objects[i];
            let b = &objects[j];

            if a.link_idx == b.link_idx {
                continue;
            }

            let pair = if a.link_idx < b.link_idx {
                (a.link_idx, b.link_idx)
            } else {
                (b.link_idx, a.link_idx)
            };

            if ignored_pairs.contains(&pair) {
                continue;
            }

            let d = query::distance(&a.world_pose, &*a.shape, &b.world_pose, &*b.shape)
                .unwrap_or(f32::INFINITY);
            min_dist = Some(match min_dist {
                Some(v) => v.min(d),
                None => d,
            });
        }
    }

    min_dist
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
    use crate::robot::{CollisionData, InertialData, JointData, LinkData};

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
        }];

        let mut link_map = HashMap::new();
        link_map.insert("base".to_string(), 0);
        link_map.insert("link1".to_string(), 1);

        let mut joint_map = HashMap::new();
        joint_map.insert("joint0".to_string(), 0);

        let mut children_joints = HashMap::new();
        children_joints.insert("base".to_string(), vec![0]);

        RobotModel {
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
        }
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
