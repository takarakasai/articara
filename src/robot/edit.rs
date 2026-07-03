//! Structural editing of [`RobotModel`]: creating empty models, adding /
//! removing links and joints, renaming, and index rebuilding.
//!
//! Format I/O lives in [`super::io`]; ray picking in [`super::pick`].
//! Slated to delegate to `misarta::native::edit` (A2 in
//! `doc/refactor_20260702.md` §4.1) once that lands.

use nalgebra as na;
use std::collections::HashMap;

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
    /// Remove a link and its entire subtree. Returns the removed link
    /// names. Delegates to [`misarta::native::edit::remove_link_in`],
    /// which also cleans up dangling references (sensors on removed
    /// links, collision pairs / loop closures touching them, mimics of
    /// removed joints, pose angle entries) — previously those went stale.
    pub fn remove_link(&mut self, link_name: &str) -> Result<Vec<String>, String> {
        let removed = misarta::native::edit::remove_link_in(self, link_name)
            .map_err(|e| e.to_string())?;
        self.rebuild_indices();
        Ok(removed)
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

    /// Rename a link. Delegates the reference fixups (joint parent/child,
    /// loop closures, sensor mounts, collision pairs, gait foot links,
    /// root) to [`misarta::native::edit::rename_link_in`] via the
    /// [`misarta::native::edit::EditTables`] impl below, then rebuilds the
    /// derived indices. Returns `true` on success.
    pub fn rename_link(&mut self, old_name: &str, new_name: &str) -> bool {
        if new_name.trim() == old_name {
            return false;
        }
        match misarta::native::edit::rename_link_in(self, old_name, new_name) {
            Ok(()) => {
                self.rebuild_indices();
                true
            }
            Err(_) => false,
        }
    }

    /// Rename a joint. Delegates the reference fixups (mimic follower /
    /// source, pose angle keys) to
    /// [`misarta::native::edit::rename_joint_in`], then rebuilds indices.
    pub fn rename_joint(&mut self, old_name: &str, new_name: &str) -> bool {
        if new_name.trim() == old_name {
            return false;
        }
        match misarta::native::edit::rename_joint_in(self, old_name, new_name) {
            Ok(()) => {
                self.rebuild_indices();
                true
            }
            Err(_) => false,
        }
    }

    /// Return a list of all link names (for UI combo boxes).
    pub fn link_names(&self) -> Vec<String> {
        self.links.iter().map(|l| l.name.clone()).collect()
    }
}

// ─── misarta EditTables: the generic edit core runs on RobotModel ──────────
//
// The invariant-preserving edit logic (validation order, reference fixups,
// dangling-row cleanup) lives once in `misarta::native::edit`; this impl
// just enumerates where RobotModel keeps its name-reference slots. Derived
// indices (`link_map` / `joint_map` / `children_joints` / misarta cache)
// are rebuilt by the calling wrappers via `rebuild_indices()`, per the
// trait contract.
impl misarta::native::edit::EditTables for RobotModel {
    fn root_link(&self) -> String {
        self.root_link.clone()
    }

    fn has_link(&self, name: &str) -> bool {
        self.link_map.contains_key(name)
    }

    fn has_joint(&self, name: &str) -> bool {
        self.joint_map.contains_key(name)
    }

    fn joints_topology(&self) -> Vec<(String, String, String)> {
        self.joints
            .iter()
            .map(|j| (j.name.clone(), j.parent_link.clone(), j.child_link.clone()))
            .collect()
    }

    fn visit_link_name_slots(&mut self, f: &mut dyn FnMut(&mut String)) {
        for l in &mut self.links {
            f(&mut l.name);
        }
        f(&mut self.root_link);
        for j in &mut self.joints {
            f(&mut j.parent_link);
            f(&mut j.child_link);
        }
        for s in &mut self.sensors {
            f(&mut s.link);
        }
        for cp in &mut self.collision_pairs {
            f(&mut cp.link_a);
            f(&mut cp.link_b);
        }
        for lc in &mut self.loop_closures {
            f(&mut lc.link_a);
            f(&mut lc.link_b);
        }
        for g in &mut self.gaits {
            f(&mut g.fl_foot);
            f(&mut g.fr_foot);
            f(&mut g.rl_foot);
            f(&mut g.rr_foot);
        }
    }

    fn visit_joint_name_slots(&mut self, f: &mut dyn FnMut(&mut String)) {
        for j in &mut self.joints {
            f(&mut j.name);
        }
        for m in &mut self.mimics {
            f(&mut m.joint);
            f(&mut m.source);
        }
        // Actuator settings are per-joint fields in `JointData`, so there
        // is no separate actuator-reference table here.
    }

    fn rekey_joint_maps(&mut self, old: &str, new: &str) {
        for p in &mut self.poses {
            if let Some(v) = p.angles.remove(old) {
                p.angles.insert(new.to_string(), v);
            }
        }
        // No home table on RobotModel — the home entry is generated from
        // live joint state at `.misa` save time.
    }

    fn remove_link_entities(&mut self, names: &[String]) {
        self.links.retain(|l| !names.contains(&l.name));
    }

    fn remove_joint_entities(&mut self, names: &[String]) {
        self.joints.retain(|j| !names.contains(&j.name));
    }

    fn retain_rows_by_link(&mut self, keep: &dyn Fn(&str) -> bool) {
        self.sensors.retain(|s| keep(&s.link));
        self.collision_pairs
            .retain(|cp| keep(&cp.link_a) && keep(&cp.link_b));
        self.loop_closures
            .retain(|lc| keep(&lc.link_a) && keep(&lc.link_b));
    }

    fn retain_rows_by_joint(&mut self, keep: &dyn Fn(&str) -> bool) {
        self.mimics
            .retain(|m| keep(&m.joint) && keep(&m.source));
        for p in &mut self.poses {
            p.angles.retain(|k, _| keep(k));
        }
    }
}
