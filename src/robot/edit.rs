//! Structural editing of [`RobotModel`]: creating empty models, adding /
//! removing links and joints, renaming, and index rebuilding.
//!
//! Format I/O lives in [`super::io`]; ray picking in [`super::pick`].
//! Slated to delegate to `misarta::native::edit` (A2 in
//! `doc/refactor_20260702.md` §4.1) once that lands.

use nalgebra as na;
use std::collections::{HashMap, HashSet};

use crate::rbd::model::*;

impl RobotModel {
    // ========== Model editing: Add / Remove links and joints ==========

    /// Create a new empty model with a single root link.
    pub fn new_empty(name: &str) -> Self {
        let root_name = "base_link".to_string();
        let mut link_map = HashMap::new();
        link_map.insert(root_name.clone(), 0);
        let mut model = Self {
            name: name.to_string(),
            links: vec![LinkData {
                name: root_name.clone(),
                visuals: vec![VisualData {
                    origin: na::Isometry3::identity(),
                    geometry: GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.025 },
                    color: [0.7, 0.7, 0.7, 1.0],
                }],
                collisions: Vec::new(),
                inertial: InertialData {
                    origin: na::Isometry3::identity(),
                    mass: 1.0,
                    ixx: 0.001, ixy: 0.0, ixz: 0.0,
                    iyy: 0.001, iyz: 0.0, izz: 0.001,
                },
                collision_enabled: true,
            }],
            joints: Vec::new(),
            link_map,
            joint_map: HashMap::new(),
            root_link: root_name,
            children_joints: HashMap::new(),
            materials: HashMap::new(),
            joint_positions: Vec::new(),
            source_path: None,
            base_transform: na::Isometry3::identity(),
            misarta_cache: None,
            loop_closures: Vec::new(),
            poses: Vec::new(),
            collision_pairs: Vec::new(),
            sequences: Vec::new(),
            mimics: Vec::new(),
            sensors: Vec::new(),
            gaits: Vec::new(),
        };
        model.rebuild_misarta_model();
        model
    }

    /// Generate a unique link name that doesn't collide with existing ones.
    pub fn generate_link_name(&self, base: &str) -> String {
        if !self.link_map.contains_key(base) {
            return base.to_string();
        }
        for i in 1.. {
            let name = format!("{base}_{i}");
            if !self.link_map.contains_key(&name) {
                return name;
            }
        }
        unreachable!()
    }

    /// Generate a unique joint name that doesn't collide with existing ones.
    pub fn generate_joint_name(&self, base: &str) -> String {
        if !self.joint_map.contains_key(base) {
            return base.to_string();
        }
        for i in 1.. {
            let name = format!("{base}_{i}");
            if !self.joint_map.contains_key(&name) {
                return name;
            }
        }
        unreachable!()
    }

    /// Add a new link with default values. Returns the index of the new link.
    pub fn add_link(&mut self, name: &str, geometry: GeomData, color: [f32; 4]) -> usize {
        let idx = self.links.len();
        self.link_map.insert(name.to_string(), idx);
        self.links.push(LinkData {
            name: name.to_string(),
            visuals: vec![VisualData {
                origin: na::Isometry3::identity(),
                geometry,
                color,
            }],
            collisions: Vec::new(),
            inertial: InertialData {
                origin: na::Isometry3::identity(),
                mass: 0.1,
                ixx: 0.0001, ixy: 0.0, ixz: 0.0,
                iyy: 0.0001, iyz: 0.0, izz: 0.0001,
            },
            collision_enabled: true,
        });
        self.misarta_cache = None; // invalidate stale cache
        idx
    }

    /// Add a new joint connecting parent_link to child_link.
    /// Returns the index of the new joint, or Err if parent/child not found.
    pub fn add_joint(
        &mut self,
        name: &str,
        joint_type: &str,
        parent_link: &str,
        child_link: &str,
        origin: na::Isometry3<f32>,
        axis: na::Vector3<f32>,
        lower: f64,
        upper: f64,
    ) -> Result<usize, String> {
        if !self.link_map.contains_key(parent_link) {
            return Err(format!("Parent link '{}' not found", parent_link));
        }
        if !self.link_map.contains_key(child_link) {
            return Err(format!("Child link '{}' not found", child_link));
        }
        let idx = self.joints.len();
        self.joint_map.insert(name.to_string(), idx);
        self.children_joints
            .entry(parent_link.to_string())
            .or_default()
            .push(idx);
        self.joints.push(JointData {
            name: name.to_string(),
            joint_type: joint_type.to_string(),
            parent_link: parent_link.to_string(),
            child_link: child_link.to_string(),
            origin,
            axis,
            lower,
            upper,
            effort: 10.0,
            velocity: 5.0,
            actuator_mode: crate::rbd::model::ActuatorMode::default(),
            actuator_kp: 50.0,
            actuator_kv: 5.0,
                    // Match `default_armature()` — see comment on the URDF
                    // loader path for the rationale.
                    armature: 0.0014,
                    joint_damping: 0.0,
        });
        self.joint_positions.push(0.0);
        self.misarta_cache = None; // invalidate stale cache
        Ok(idx)
    }

    /// Add a child link + joint pair in one step.
    /// Creates a new link, then a joint connecting parent → new link.
    /// Returns (link_index, joint_index).
    pub fn add_child(
        &mut self,
        parent_link: &str,
        link_name: &str,
        joint_name: &str,
        joint_type: &str,
        origin: na::Isometry3<f32>,
        axis: na::Vector3<f32>,
        geometry: GeomData,
        color: [f32; 4],
        lower: f64,
        upper: f64,
    ) -> Result<(usize, usize), String> {
        let li = self.add_link(link_name, geometry, color);
        let ji = self.add_joint(joint_name, joint_type, parent_link, link_name, origin, axis, lower, upper)?;
        Ok((li, ji))
    }

    /// Remove a link and all joints that reference it (parent or child).
    /// Also removes child links recursively. Returns the names of removed links.
    pub fn remove_link(&mut self, link_name: &str) -> Result<Vec<String>, String> {
        if link_name == self.root_link {
            return Err("Cannot remove the root link".to_string());
        }
        if !self.link_map.contains_key(link_name) {
            return Err(format!("Link '{}' not found", link_name));
        }

        // Collect all links to remove (this link + all descendants)
        let mut to_remove = Vec::new();
        self.collect_descendants(link_name, &mut to_remove);

        // Remove joints that reference any of the removed links
        let remove_set: HashSet<String> = to_remove.iter().cloned().collect();
        self.joints.retain(|j| {
            !remove_set.contains(&j.parent_link) || !remove_set.contains(&j.child_link)
        });
        // Also remove the joint whose child is link_name
        self.joints.retain(|j| !remove_set.contains(&j.child_link));

        // Remove the links themselves
        self.links.retain(|l| !remove_set.contains(&l.name));

        // Rebuild indices
        self.rebuild_indices();
        Ok(to_remove)
    }

    /// Collect a link and all its descendants.
    fn collect_descendants(&self, link_name: &str, result: &mut Vec<String>) {
        result.push(link_name.to_string());
        if let Some(child_joints) = self.children_joints.get(link_name) {
            for &ji in child_joints {
                let child = &self.joints[ji].child_link;
                self.collect_descendants(child, result);
            }
        }
    }

    /// Rebuild all index maps after structural changes (add/remove).
    pub fn rebuild_indices(&mut self) {
        self.link_map.clear();
        for (i, link) in self.links.iter().enumerate() {
            self.link_map.insert(link.name.clone(), i);
        }
        self.joint_map.clear();
        self.children_joints.clear();
        for (i, joint) in self.joints.iter().enumerate() {
            self.joint_map.insert(joint.name.clone(), i);
            self.children_joints
                .entry(joint.parent_link.clone())
                .or_default()
                .push(i);
        }
        // Fix joint_positions length
        self.joint_positions.resize(self.joints.len(), 0.0);
        self.rebuild_misarta_model();
    }

    /// Rename a link.  Updates the canonical name, all joint parent/child
    /// references, loop-closure references, and rebuilds derived indices.
    /// Returns `true` on success, `false` if `new_name` is empty or already taken.
    pub fn rename_link(&mut self, old_name: &str, new_name: &str) -> bool {
        let new_name = new_name.trim();
        if new_name.is_empty() || new_name == old_name {
            return false;
        }
        // Reject duplicates
        if self.link_map.contains_key(new_name) {
            return false;
        }
        // Find link index
        let Some(&li) = self.link_map.get(old_name) else {
            return false;
        };
        // 1. Rename the link itself
        self.links[li].name = new_name.to_string();
        // 2. Update root_link
        if self.root_link == old_name {
            self.root_link = new_name.to_string();
        }
        // 3. Update all joints referencing this link
        for joint in &mut self.joints {
            if joint.parent_link == old_name {
                joint.parent_link = new_name.to_string();
            }
            if joint.child_link == old_name {
                joint.child_link = new_name.to_string();
            }
        }
        // 4. Update loop-closure references
        for lc in &mut self.loop_closures {
            if lc.link_a == old_name {
                lc.link_a = new_name.to_string();
            }
            if lc.link_b == old_name {
                lc.link_b = new_name.to_string();
            }
        }
        // 5. Sensor mounts, collision pairs and gait foot links reference
        //    links by name too (same invariant set as
        //    `misarta::native::edit::rename_link`).
        for s in &mut self.sensors {
            if s.link == old_name {
                s.link = new_name.to_string();
            }
        }
        for cp in &mut self.collision_pairs {
            if cp.link_a == old_name {
                cp.link_a = new_name.to_string();
            }
            if cp.link_b == old_name {
                cp.link_b = new_name.to_string();
            }
        }
        for g in &mut self.gaits {
            for foot in [
                &mut g.fl_foot,
                &mut g.fr_foot,
                &mut g.rl_foot,
                &mut g.rr_foot,
            ] {
                if foot == old_name {
                    *foot = new_name.to_string();
                }
            }
        }
        // 6. Rebuild all derived maps
        self.rebuild_indices();
        true
    }

    /// Rename a joint.  Updates the canonical name and rebuilds derived indices.
    /// Returns `true` on success, `false` if `new_name` is empty or already taken.
    pub fn rename_joint(&mut self, old_name: &str, new_name: &str) -> bool {
        let new_name = new_name.trim();
        if new_name.is_empty() || new_name == old_name {
            return false;
        }
        if self.joint_map.contains_key(new_name) {
            return false;
        }
        let Some(&ji) = self.joint_map.get(old_name) else {
            return false;
        };
        self.joints[ji].name = new_name.to_string();
        // Mimics and pose angle maps reference joints by name (same
        // invariant set as `misarta::native::edit::rename_joint`).
        for m in &mut self.mimics {
            if m.joint == old_name {
                m.joint = new_name.to_string();
            }
            if m.source == old_name {
                m.source = new_name.to_string();
            }
        }
        for p in &mut self.poses {
            if let Some(v) = p.angles.remove(old_name) {
                p.angles.insert(new_name.to_string(), v);
            }
        }
        self.rebuild_indices();
        true
    }

    /// Return a list of all link names (for UI combo boxes).
    pub fn link_names(&self) -> Vec<String> {
        self.links.iter().map(|l| l.name.clone()).collect()
    }
}
