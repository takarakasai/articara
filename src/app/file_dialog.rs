//! Lightweight in-app file browser dialog built on egui::Window.
//!
//! Provides a reusable `FileDialog` that shows an interactive directory
//! listing, path text input, and OK / Cancel buttons.  No native OS
//! dialog or external crate needed.

use eframe::egui;
use std::path::{Path, PathBuf};

/// The purpose of the dialog (affects title & filtering).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FileDialogMode {
    /// Choose a file to open.
    Open,
    /// Choose a path to save to.
    Save,
    /// Choose a directory.
    ChooseDir,
}

/// Outcome after drawing one frame of the dialog.
#[derive(Clone, Debug)]
pub enum FileDialogResult {
    /// Dialog is still open — user hasn't decided yet.
    Pending,
    /// User confirmed a path.
    Confirmed(PathBuf),
    /// User cancelled.
    Cancelled,
}

/// Persistent state for one file dialog instance.
pub struct FileDialog {
    /// Is the dialog currently open?
    pub open: bool,
    /// Title shown in the window title bar.
    pub title: String,
    /// Mode of the dialog.
    pub mode: FileDialogMode,
    /// Current directory being browsed.
    current_dir: PathBuf,
    /// Text input for the full path / filename.
    path_input: String,
    /// File extension filter (empty = no filter).
    extension_filter: Vec<String>,
    /// Cached listing of the current directory.
    cached_entries: Vec<DirEntry>,
    /// Whether the cache needs refreshing.
    dirty: bool,
    /// Unique id salt for egui.
    id_salt: String,
}

#[derive(Clone)]
struct DirEntry {
    name: String,
    is_dir: bool,
}

impl FileDialog {
    /// Create a new file dialog with the given id (must be unique per dialog instance).
    pub fn new(id: &str) -> Self {
        Self {
            open: false,
            title: "Select file".into(),
            mode: FileDialogMode::Open,
            current_dir: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")),
            path_input: String::new(),
            extension_filter: Vec::new(),
            cached_entries: Vec::new(),
            dirty: true,
            id_salt: id.to_string(),
        }
    }

    /// Open the dialog, optionally starting at `start_path`.
    /// If `start_path` is a file, the directory is used and the filename
    /// pre-fills the text input.
    pub fn open(&mut self, title: &str, mode: FileDialogMode, start_path: Option<&Path>, extensions: &[&str]) {
        self.open = true;
        self.title = title.into();
        self.mode = mode;
        self.extension_filter = extensions.iter().map(|e| e.to_string()).collect();

        if let Some(p) = start_path {
            if p.is_dir() {
                self.current_dir = p.to_path_buf();
                self.path_input = p.display().to_string();
            } else {
                if let Some(parent) = p.parent() {
                    if parent.is_dir() {
                        self.current_dir = parent.to_path_buf();
                    }
                }
                self.path_input = p.display().to_string();
            }
        }
        self.dirty = true;
    }

    /// Draw the dialog. Returns the outcome this frame.
    pub fn show(&mut self, ctx: &egui::Context) -> FileDialogResult {
        if !self.open {
            return FileDialogResult::Pending;
        }

        // Refresh directory listing if needed
        if self.dirty {
            self.refresh_entries();
            self.dirty = false;
        }

        let mut result = FileDialogResult::Pending;

        let mut still_open = self.open;
        egui::Window::new(&self.title)
            .id(egui::Id::new(&self.id_salt))
            .open(&mut still_open)
            .resizable(true)
            .default_width(450.0)
            .default_height(350.0)
            .show(ctx, |ui| {
                // --- Current directory breadcrumb ---
                ui.horizontal(|ui| {
                    ui.label("📁");
                    let dir_str = self.current_dir.display().to_string();
                    ui.label(
                        egui::RichText::new(&dir_str)
                            .monospace()
                            .small(),
                    );
                    if ui.small_button("⬆").on_hover_text("Parent directory").clicked() {
                        if let Some(parent) = self.current_dir.parent() {
                            self.current_dir = parent.to_path_buf();
                            self.dirty = true;
                        }
                    }
                });

                ui.separator();

                // --- Directory listing ---
                let available_h = (ui.available_height() - 70.0).max(80.0);
                egui::ScrollArea::vertical()
                    .max_height(available_h)
                    .show(ui, |ui| {
                        // Parent dir entry
                        if self.current_dir.parent().is_some() {
                            if ui.selectable_label(false, "📁 ..").clicked() {
                                if let Some(parent) = self.current_dir.parent() {
                                    self.current_dir = parent.to_path_buf();
                                    self.dirty = true;
                                }
                            }
                        }

                        let entries = self.cached_entries.clone();
                        for entry in &entries {
                            let icon = if entry.is_dir { "📁 " } else { "📄 " };
                            let label = format!("{}{}", icon, entry.name);

                            if ui.selectable_label(false, &label).clicked() {
                                let full = self.current_dir.join(&entry.name);
                                if entry.is_dir {
                                    self.current_dir = full;
                                    self.dirty = true;
                                    if self.mode == FileDialogMode::ChooseDir {
                                        self.path_input = self.current_dir.display().to_string();
                                    }
                                } else {
                                    self.path_input = full.display().to_string();
                                }
                            }
                        }
                    });

                ui.separator();

                // --- Path input ---
                ui.horizontal(|ui| {
                    let label = match self.mode {
                        FileDialogMode::Open => "File:",
                        FileDialogMode::Save => "Save as:",
                        FileDialogMode::ChooseDir => "Directory:",
                    };
                    ui.label(label);
                    let te = ui.add_sized(
                        egui::vec2(ui.available_width() - 4.0, 20.0),
                        egui::TextEdit::singleline(&mut self.path_input),
                    );
                    // If user edited the path and it points to a valid dir, refresh
                    if te.changed() {
                        let p = PathBuf::from(&self.path_input);
                        if p.is_dir() {
                            self.current_dir = p;
                            self.dirty = true;
                        } else if let Some(parent) = p.parent() {
                            if parent.is_dir() && parent != self.current_dir.as_path() {
                                self.current_dir = parent.to_path_buf();
                                self.dirty = true;
                            }
                        }
                    }
                });

                // --- OK / Cancel buttons ---
                ui.horizontal(|ui| {
                    let ok_label = match self.mode {
                        FileDialogMode::Open => "Open",
                        FileDialogMode::Save => "Save",
                        FileDialogMode::ChooseDir => "Select",
                    };
                    let ok_enabled = !self.path_input.is_empty();
                    if ui.add_enabled(ok_enabled, egui::Button::new(ok_label)).clicked() {
                        let path = PathBuf::from(&self.path_input);
                        result = FileDialogResult::Confirmed(path);
                        self.open = false;
                    }
                    if ui.button("Cancel").clicked() {
                        result = FileDialogResult::Cancelled;
                        self.open = false;
                    }
                });
            });

        // Window close button (X) was clicked
        if !still_open {
            self.open = false;
            if matches!(result, FileDialogResult::Pending) {
                result = FileDialogResult::Cancelled;
            }
        }

        result
    }

    fn refresh_entries(&mut self) {
        self.cached_entries.clear();
        let Ok(read_dir) = std::fs::read_dir(&self.current_dir) else {
            return;
        };

        let mut dirs = Vec::new();
        let mut files = Vec::new();

        for entry in read_dir.flatten() {
            let Ok(meta) = entry.metadata() else { continue };
            let name = entry.file_name().to_string_lossy().into_owned();

            // Skip hidden files
            if name.starts_with('.') {
                continue;
            }

            if meta.is_dir() {
                dirs.push(DirEntry { name, is_dir: true });
            } else {
                // Apply extension filter (case-insensitive)
                if !self.extension_filter.is_empty() {
                    let name_lower = name.to_ascii_lowercase();
                    let has_ext = self.extension_filter.iter().any(|ext| {
                        name_lower.ends_with(&format!(".{ext}"))
                    });
                    if !has_ext {
                        continue;
                    }
                }
                files.push(DirEntry { name, is_dir: false });
            }
        }

        dirs.sort_by(|a, b| a.name.to_lowercase().cmp(&b.name.to_lowercase()));
        files.sort_by(|a, b| a.name.to_lowercase().cmp(&b.name.to_lowercase()));

        self.cached_entries.extend(dirs);
        self.cached_entries.extend(files);
    }
}
