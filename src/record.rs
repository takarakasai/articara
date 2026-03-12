//! Offscreen recording of the MuJoCo sim to PNG frames, via mujoco-rs's renderer
//! (EGL headless, winit fallback under a virtual display like `xvfb-run`). Pair
//! the saved frames with `ffmpeg` to make an mp4. Only compiled with `record`.
//!
//! ```ignore
//! let mut rec = Recorder::new(&sim, 640, 480, "base", 1.6, 90.0, -15.0, "/tmp/frames")?;
//! // each video frame:
//! rec.capture(&mut sim)?;
//! // then: ffmpeg -framerate 30 -i /tmp/frames/%05d.png out.mp4
//! ```

use std::path::PathBuf;
use std::sync::Arc;

use mujoco::prelude::{MjModel, MjtObj, MjvCamera};
use mujoco::renderer::MjRenderer;

use crate::mujoco_sim::MujocoSim;

/// Owns an offscreen MuJoCo renderer + a tracking camera and writes one PNG per
/// captured frame into `dir` (`00000.png`, `00001.png`, …).
pub struct Recorder {
    renderer: MjRenderer,
    dir: PathBuf,
    count: usize,
}

impl Recorder {
    /// Create a recorder with a camera tracking `track_body`. `dist`/`az`/`el`
    /// are the camera distance (m), azimuth and elevation (deg).
    pub fn new(
        sim: &MujocoSim,
        width: usize,
        height: usize,
        track_body: &str,
        dist: f64,
        az: f64,
        el: f64,
        dir: &str,
    ) -> Result<Self, String> {
        let model = sim.model_arc();
        let track_id = model
            .name_to_id(MjtObj::mjOBJ_BODY, track_body)
            .ok_or_else(|| format!("record: body {track_body:?} not found"))?;
        let mut cam = MjvCamera::new_tracking(track_id);
        cam.distance = dist;
        cam.azimuth = az;
        cam.elevation = el;
        let renderer = MjRenderer::new(model, width, height, 0)
            .map_err(|e| format!("record: renderer init failed: {e:?}"))?
            .with_camera(cam);
        std::fs::create_dir_all(dir).map_err(|e| format!("record: mkdir {dir}: {e}"))?;
        Ok(Self { renderer, dir: PathBuf::from(dir), count: 0 })
    }

    /// Render the current sim state and save it as the next PNG frame.
    pub fn capture(&mut self, sim: &mut MujocoSim) -> Result<(), String> {
        self.renderer
            .sync_data(sim.data_mut())
            .map_err(|e| format!("record: sync_data: {e:?}"))?;
        self.renderer
            .render()
            .map_err(|e| format!("record: render: {e:?}"))?;
        let path = self.dir.join(format!("{:05}.png", self.count));
        self.renderer
            .save_rgb(&path)
            .map_err(|e| format!("record: save_rgb: {e:?}"))?;
        self.count += 1;
        Ok(())
    }

    /// Number of frames captured so far.
    pub fn frame_count(&self) -> usize {
        self.count
    }

    /// Directory the PNG frames are written to.
    pub fn dir(&self) -> &std::path::Path {
        &self.dir
    }
}
