//! Undo / redo history for robot model editing.
//!
//! Uses a snapshot-based approach: before each undoable operation the full
//! `RobotModel` is cloned and stored.  Consecutive edits with the same
//! description are merged so that, e.g., dragging a slider produces a single
//! undo entry rather than one per frame.

use crate::robot::RobotModel;

/// A single entry in the undo/redo stack.
#[derive(Clone)]
pub struct HistoryEntry {
    /// Human-readable description of the operation.
    pub description: String,
    /// Snapshot of the model state **before** the operation.
    model: RobotModel,
}

/// Manages the undo/redo stacks and the operation log.
pub struct History {
    undo_stack: Vec<HistoryEntry>,
    redo_stack: Vec<HistoryEntry>,
    /// Description of the currently active (continuous) edit.
    /// While this matches, subsequent `record()` calls are merged.
    active_edit: Option<String>,
    /// Maximum number of undo entries to keep.
    max_entries: usize,
    /// Complete log of operations (not limited like the undo stack).
    log: Vec<String>,
}

impl History {
    /// Create a new empty history with the given capacity.
    pub fn new(max_entries: usize) -> Self {
        Self {
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            active_edit: None,
            max_entries,
            log: Vec::new(),
        }
    }

    /// Clear all history (e.g. when loading a new model).
    pub fn clear(&mut self) {
        self.undo_stack.clear();
        self.redo_stack.clear();
        self.active_edit = None;
        self.log.clear();
    }

    /// Record a model snapshot before an edit.
    ///
    /// If `desc` matches the active edit description the call is *merged*
    /// (no duplicate push) — this handles continuous edits like dragging a
    /// slider across multiple frames.
    pub fn record(&mut self, desc: &str, model: RobotModel) {
        if self.active_edit.as_deref() == Some(desc) {
            // Same ongoing edit — the first call already captured the
            // "before" state.
            return;
        }
        self.undo_stack.push(HistoryEntry {
            description: desc.to_string(),
            model,
        });
        self.redo_stack.clear();
        self.active_edit = Some(desc.to_string());
        self.log.push(desc.to_string());
        self.trim();
    }

    /// End the current edit phase so that the next `record()` with the
    /// same description will push a new entry.
    pub fn finalize(&mut self) {
        self.active_edit = None;
    }

    /// Undo the last operation, replacing `current` with the previous
    /// model state.  Returns the description of the undone operation.
    pub fn undo(&mut self, current: &mut RobotModel) -> Option<String> {
        self.finalize();
        let entry = self.undo_stack.pop()?;
        let desc = entry.description.clone();
        let old_current = std::mem::replace(current, entry.model);
        self.redo_stack.push(HistoryEntry {
            description: desc.clone(),
            model: old_current,
        });
        Some(desc)
    }

    /// Redo the last undone operation.  Returns the description.
    pub fn redo(&mut self, current: &mut RobotModel) -> Option<String> {
        self.finalize();
        let entry = self.redo_stack.pop()?;
        let desc = entry.description.clone();
        let old_current = std::mem::replace(current, entry.model);
        self.undo_stack.push(HistoryEntry {
            description: desc.clone(),
            model: old_current,
        });
        Some(desc)
    }

    /// Whether an undo operation is available.
    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    /// Whether a redo operation is available.
    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    /// Number of stored undo entries.
    pub fn undo_count(&self) -> usize {
        self.undo_stack.len()
    }

    /// Number of stored redo entries.
    pub fn redo_count(&self) -> usize {
        self.redo_stack.len()
    }

    /// Peek at the description of the operation that would be undone.
    pub fn undo_description(&self) -> Option<&str> {
        self.undo_stack.last().map(|e| e.description.as_str())
    }

    /// Peek at the description of the operation that would be redone.
    pub fn redo_description(&self) -> Option<&str> {
        self.redo_stack.last().map(|e| e.description.as_str())
    }

    /// Read-only access to the operation log.
    pub fn log(&self) -> &[String] {
        &self.log
    }

    /// Read-only access to the undo stack entries (oldest first).
    pub fn undo_entries(&self) -> &[HistoryEntry] {
        &self.undo_stack
    }

    /// Read-only access to the redo stack entries.
    /// The *last* element is the next entry to be redone.
    pub fn redo_entries(&self) -> &[HistoryEntry] {
        &self.redo_stack
    }

    /// Jump to a specific position in the timeline.
    ///
    /// `target_pos` is the desired undo-stack length after the jump.
    /// Range: `0` (before all edits) …  `undo_count + redo_count`
    /// (after all edits including redone ones).  Returns the last
    /// description processed, if any.
    pub fn goto(&mut self, target_pos: usize, current: &mut RobotModel) -> Option<String> {
        self.finalize();
        let mut last_desc: Option<String> = None;
        let cur = self.undo_stack.len();
        if target_pos < cur {
            for _ in 0..(cur - target_pos) {
                last_desc = self.undo(current);
            }
        } else if target_pos > cur {
            for _ in 0..(target_pos - cur) {
                last_desc = self.redo(current);
            }
        }
        last_desc
    }

    fn trim(&mut self) {
        while self.undo_stack.len() > self.max_entries {
            self.undo_stack.remove(0);
        }
    }
}

// =========================================================================
//  Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_model(name: &str) -> RobotModel {
        let mut m = RobotModel::new_empty(name);
        m.joint_positions = vec![0.0; 3];
        m
    }

    #[test]
    fn record_and_undo() {
        let mut h = History::new(50);
        let mut model = make_model("r");
        model.joint_positions[0] = 0.0;

        h.record("Set joint 0", model.clone());
        model.joint_positions[0] = 1.0;

        assert!(h.can_undo());
        let desc = h.undo(&mut model).unwrap();
        assert_eq!(desc, "Set joint 0");
        assert_eq!(model.joint_positions[0], 0.0);
    }

    #[test]
    fn redo_after_undo() {
        let mut h = History::new(50);
        let mut model = make_model("r");

        h.record("Edit", model.clone());
        model.joint_positions[0] = 1.0;

        h.undo(&mut model);
        assert!(h.can_redo());
        let desc = h.redo(&mut model).unwrap();
        assert_eq!(desc, "Edit");
        assert_eq!(model.joint_positions[0], 1.0);
    }

    #[test]
    fn new_edit_clears_redo() {
        let mut h = History::new(50);
        let mut model = make_model("r");

        h.record("A", model.clone());
        model.joint_positions[0] = 1.0;

        h.undo(&mut model);
        assert!(h.can_redo());

        // New edit clears redo
        h.record("B", model.clone());
        model.joint_positions[0] = 2.0;
        assert!(!h.can_redo());
    }

    #[test]
    fn merge_same_description() {
        let mut h = History::new(50);
        let mut model = make_model("r");

        h.record("Drag", model.clone());
        model.joint_positions[0] = 0.5;
        // Same description → merged, no new push
        h.record("Drag", model.clone());
        model.joint_positions[0] = 1.0;
        h.record("Drag", model.clone());
        model.joint_positions[0] = 1.5;

        assert_eq!(h.undo_count(), 1);
        // Undo reverts to state before all three "Drag" calls
        h.undo(&mut model);
        assert_eq!(model.joint_positions[0], 0.0);
    }

    #[test]
    fn finalize_breaks_merge() {
        let mut h = History::new(50);
        let mut model = make_model("r");

        h.record("Drag", model.clone());
        model.joint_positions[0] = 1.0;
        h.finalize();

        h.record("Drag", model.clone());
        model.joint_positions[0] = 2.0;

        assert_eq!(h.undo_count(), 2);
    }

    #[test]
    fn trim_respects_max() {
        let mut h = History::new(3);
        let model = make_model("r");

        for i in 0..5 {
            h.finalize();
            h.record(&format!("Op {i}"), model.clone());
        }
        assert!(h.undo_count() <= 3);
    }

    #[test]
    fn log_records_all() {
        let mut h = History::new(50);
        let model = make_model("r");

        h.record("A", model.clone());
        h.finalize();
        h.record("B", model.clone());
        h.finalize();
        h.record("C", model.clone());

        assert_eq!(h.log(), &["A", "B", "C"]);
    }

    #[test]
    fn empty_undo_returns_none() {
        let mut h = History::new(50);
        let mut model = make_model("r");
        assert!(h.undo(&mut model).is_none());
    }

    #[test]
    fn empty_redo_returns_none() {
        let mut h = History::new(50);
        let mut model = make_model("r");
        assert!(h.redo(&mut model).is_none());
    }

    #[test]
    fn clear_removes_everything() {
        let mut h = History::new(50);
        let model = make_model("r");
        h.record("A", model.clone());
        h.clear();
        assert!(!h.can_undo());
        assert!(!h.can_redo());
        assert!(h.log().is_empty());
    }

    #[test]
    fn goto_jumps_to_target() {
        let mut h = History::new(50);
        let mut model = make_model("r");

        // Build 3 edits: jp[0] = 0 → 1 → 2 → 3
        h.record("A", model.clone());
        model.joint_positions[0] = 1.0;
        h.finalize();

        h.record("B", model.clone());
        model.joint_positions[0] = 2.0;
        h.finalize();

        h.record("C", model.clone());
        model.joint_positions[0] = 3.0;
        h.finalize();

        // current pos = 3, model value = 3.0
        assert_eq!(h.undo_count(), 3);

        // goto pos 1 → after edit A, value = 1.0
        h.goto(1, &mut model);
        assert_eq!(model.joint_positions[0], 1.0);
        assert_eq!(h.undo_count(), 1);
        assert_eq!(h.redo_count(), 2);

        // goto pos 3 → after edit C, value = 3.0
        h.goto(3, &mut model);
        assert_eq!(model.joint_positions[0], 3.0);
        assert_eq!(h.undo_count(), 3);
        assert_eq!(h.redo_count(), 0);

        // goto pos 0 → before all edits, value = 0.0
        h.goto(0, &mut model);
        assert_eq!(model.joint_positions[0], 0.0);
        assert_eq!(h.undo_count(), 0);
        assert_eq!(h.redo_count(), 3);
    }
}
