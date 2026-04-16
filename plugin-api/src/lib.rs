//! Shared types for the articara WASM plugin protocol.
//!
//! This crate defines:
//! - **View primitives** — a fixed set of UI building blocks that the host
//!   generically renders (headings, scalars, tables, plots, bars, …).
//! - **Request / Response envelopes** — the JSON wire format between host and
//!   WASM plugin.
//!
//! Both the WASM plugin and the host-side runner depend on this crate so that
//! the protocol is defined in a single place.

use serde::{Deserialize, Serialize};

// ======================================================================
//  Request / Response envelopes
// ======================================================================

/// Request envelope sent from the host to the WASM plugin.
#[derive(Debug, Serialize, Deserialize)]
pub struct Request {
    /// Protocol version (currently `1`).
    pub version: u32,
    /// Command name (e.g. `"jump_sim"`, `"static_analysis"`).
    pub command: String,
    /// Command-specific parameters (opaque JSON object).
    pub params: serde_json::Value,
}

/// Response envelope returned from the WASM plugin to the host.
#[derive(Debug, Serialize, Deserialize)]
pub struct Response {
    /// Protocol version (matches request).
    pub version: u32,
    /// Whether the command succeeded.
    pub ok: bool,
    /// Echo of the command name.
    pub command: String,
    /// Error message when `ok == false`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Ordered list of UI views for the host to render.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub views: Option<Vec<View>>,
    /// Machine-readable raw data (command-specific, for programmatic use).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<serde_json::Value>,
}

impl Response {
    /// Convenience constructor for a successful response.
    pub fn ok(command: impl Into<String>, views: Vec<View>, data: serde_json::Value) -> Self {
        Self {
            version: 1,
            ok: true,
            command: command.into(),
            error: None,
            views: Some(views),
            data: Some(data),
        }
    }

    /// Convenience constructor for an error response.
    pub fn err(command: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            version: 1,
            ok: false,
            command: command.into(),
            error: Some(message.into()),
            views: None,
            data: None,
        }
    }
}

// ======================================================================
//  View primitives
// ======================================================================

/// A UI building block that the host renders generically.
///
/// The WASM plugin returns an ordered `Vec<View>` which the host
/// renders top-to-bottom.  Adding new View variants requires a host
/// update, but adding new *commands* does **not**.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum View {
    /// Section heading.
    Heading {
        text: String,
        /// 1 = top-level, 2 = sub-section, etc.
        level: u8,
    },

    /// Key-value scalar display (label + formatted value).
    Scalars {
        #[serde(skip_serializing_if = "Option::is_none")]
        title: Option<String>,
        items: Vec<ScalarItem>,
    },

    /// Tabular data with typed cells.
    Table {
        #[serde(skip_serializing_if = "Option::is_none")]
        title: Option<String>,
        columns: Vec<Column>,
        rows: Vec<Vec<Cell>>,
    },

    /// Multi-series line plot.
    LinePlot {
        title: String,
        x_label: String,
        y_label: String,
        series: Vec<Series>,
    },

    /// Named bar chart (e.g. torque utilisation).
    BarChart {
        title: String,
        bars: Vec<Bar>,
        /// Optional upper bound for the axis (e.g. 1.5 for 150%).
        #[serde(skip_serializing_if = "Option::is_none")]
        max_value: Option<f64>,
    },

    /// Progress indicator.
    Progress {
        label: String,
        /// 0.0 – 1.0
        value: f32,
        #[serde(skip_serializing_if = "Option::is_none")]
        text: Option<String>,
    },

    /// Log / message block.
    Log {
        messages: Vec<LogEntry>,
    },
}

// ── Sub-types ──────────────────────────────────────────────────────

/// A single key-value pair shown in a `Scalars` view.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScalarItem {
    /// Display label (e.g. "Reached Height").
    pub label: String,
    /// Formatted display value (e.g. "0.1353 m").
    pub value: String,
    /// Machine-readable numeric value for sorting / comparison.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub numeric: Option<f64>,
    /// Visual emphasis: `"primary"`, `"secondary"`, `"warning"`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub emphasis: Option<String>,
}

/// Column definition for a `Table` view.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Column {
    pub name: String,
    /// `"left"`, `"right"`, or `"center"`.  Default: `"left"`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub align: Option<String>,
}

/// A single cell in a table row.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum Cell {
    /// Plain text.
    Text { value: String },
    /// Numeric value with optional printf-style format (e.g. `".3f"`).
    Number {
        value: f64,
        #[serde(skip_serializing_if = "Option::is_none")]
        format: Option<String>,
    },
    /// Coloured tag / badge.
    Tag {
        value: String,
        /// Named colour: `"green"`, `"yellow"`, `"red"`, `"gray"`.
        #[serde(skip_serializing_if = "Option::is_none")]
        color: Option<String>,
    },
}

/// One data series in a `LinePlot`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Series {
    pub name: String,
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    /// CSS-style colour (`"#FF6464"`) or named colour.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
}

/// One bar in a `BarChart`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bar {
    pub label: String,
    pub value: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    /// Auxiliary text shown next to the bar (e.g. "hold", "drive").
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tag: Option<String>,
}

/// A single message in a `Log` view.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogEntry {
    /// `"info"`, `"warning"`, or `"error"`.
    pub level: String,
    pub text: String,
}

// ======================================================================
//  Command metadata (returned by `list_commands`)
// ======================================================================

/// Describes one available command (returned by `list_commands`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandInfo {
    pub name: String,
    pub description: String,
    /// Grouping category (e.g. `"simulation"`, `"analysis"`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<String>,
}
