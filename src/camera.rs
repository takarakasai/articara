use nalgebra as na;

/// Which camera the main viewport renders. `Free` is the original
/// user-controlled orbit camera; `Tps` follows a chosen body link and
/// gives a third-person-shooter-style trailing view.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraMode {
    Free,
    Tps,
}

impl CameraMode {
    pub const ALL: [CameraMode; 2] = [CameraMode::Free, CameraMode::Tps];
    pub fn label(self) -> &'static str {
        match self {
            CameraMode::Free => "Free orbit",
            CameraMode::Tps => "TPS (follow)",
        }
    }
}

/// Settings for the third-person follow camera. Each frame the host
/// derives a concrete [`OrbitCamera`] from these values plus the live
/// pose of the followed link.
///
/// Convention: the camera sits a fixed `distance` away from
/// `target_local_offset` (in the followed link's frame), at world-frame
/// `yaw_offset` rotation (added on top of the link's yaw so the camera
/// can swing around the body) and `pitch_offset` elevation.
#[derive(Clone, Debug)]
pub struct TpsSettings {
    /// Link to follow. `None` falls back to the model's root link.
    pub follow_link: Option<String>,
    /// Offset from the followed link's origin to the camera's look-at
    /// point, in the link's local frame. Defaults to zero (look at the
    /// link origin); raise z to look at the body's chest, etc.
    pub target_local_offset: na::Vector3<f32>,
    /// Distance from look-at to camera eye (m).
    pub distance: f32,
    /// Additional yaw rotation around the body (rad). `0` = camera
    /// directly behind the body's local +x axis. Mouse drag in TPS
    /// mode updates this.
    pub yaw_offset: f32,
    /// Camera elevation angle (rad). `0` = level with target; `+pi/4`
    /// = looking down at ~45°.
    pub pitch_offset: f32,
}

impl Default for TpsSettings {
    fn default() -> Self {
        Self {
            follow_link: None,
            target_local_offset: na::Vector3::new(0.0, 0.0, 0.0),
            distance: 1.2,
            yaw_offset: 0.0,
            pitch_offset: 0.35, // ~20°, slight downward
        }
    }
}

#[derive(Clone)]
pub struct OrbitCamera {
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub target: na::Point3<f32>,
    pub fov_y: f32,
    pub near: f32,
    pub far: f32,
}

impl OrbitCamera {
    pub fn new() -> Self {
        Self {
            yaw: std::f32::consts::FRAC_PI_4,
            pitch: 0.5,
            distance: 0.8,
            target: na::Point3::new(0.0, 0.0, 0.05),
            fov_y: 45.0_f32.to_radians(),
            near: 0.001,
            far: 100.0,
        }
    }

    pub fn eye(&self) -> na::Point3<f32> {
        let x = self.distance * self.pitch.cos() * self.yaw.cos();
        let y = self.distance * self.pitch.cos() * self.yaw.sin();
        let z = self.distance * self.pitch.sin();
        na::Point3::new(self.target.x + x, self.target.y + y, self.target.z + z)
    }

    pub fn view_matrix(&self) -> na::Matrix4<f32> {
        na::Matrix4::look_at_rh(&self.eye(), &self.target, &na::Vector3::z())
    }

    /// World-frame screen-right unit vector (i.e. the world direction that
    /// maps to "+x on the rendered image"). Derived directly from the view
    /// matrix's first row so it stays consistent with whatever look_at_rh
    /// produced — useful for screen-plane IK projections, gizmo orientation,
    /// etc. that need to match what the user actually sees.
    pub fn world_right(&self) -> na::Vector3<f32> {
        let view = self.view_matrix();
        // The view matrix rotates world → camera. Its rows (in the upper-left
        // 3×3 block) are the camera basis vectors expressed in world frame.
        na::Vector3::new(view[(0, 0)], view[(0, 1)], view[(0, 2)])
    }

    /// World-frame screen-up unit vector (the world direction mapping to
    /// "+y on the rendered image" — i.e. up on screen).
    pub fn world_up_screen(&self) -> na::Vector3<f32> {
        let view = self.view_matrix();
        na::Vector3::new(view[(1, 0)], view[(1, 1)], view[(1, 2)])
    }

    pub fn projection_matrix(&self, aspect: f32) -> na::Matrix4<f32> {
        na::Perspective3::new(aspect, self.fov_y, self.near, self.far).to_homogeneous()
    }

    /// Project a 3D world point to normalized screen coordinates [0..1, 0..1] (top-left origin).
    pub fn project(&self, world_pos: &na::Point3<f32>, aspect: f32) -> Option<na::Point2<f32>> {
        let vp = self.projection_matrix(aspect) * self.view_matrix();
        let clip = vp * na::Vector4::new(world_pos.x, world_pos.y, world_pos.z, 1.0);
        if clip.w.abs() < 1e-10 {
            return None;
        }
        let ndc = clip.xyz() / clip.w;
        // NDC: [-1,1] -> screen [0,1] with Y flipped (top=0)
        Some(na::Point2::new(
            (ndc.x + 1.0) * 0.5,
            (1.0 - ndc.y) * 0.5,
        ))
    }

    /// Cast a ray from screen coordinates (normalized [0..1]) into the scene.
    /// Returns (ray_origin, ray_direction) in world space.
    pub fn screen_ray(
        &self,
        screen_ndc: na::Point2<f32>,
        aspect: f32,
    ) -> (na::Point3<f32>, na::Vector3<f32>) {
        let view = self.view_matrix();
        let proj = self.projection_matrix(aspect);
        let vp_inv = (proj * view)
            .try_inverse()
            .unwrap_or(na::Matrix4::identity());

        // Convert [0..1] to NDC [-1..1], with Y flipped
        let ndc_x = screen_ndc.x * 2.0 - 1.0;
        let ndc_y = 1.0 - screen_ndc.y * 2.0;

        let near_clip = vp_inv * na::Vector4::new(ndc_x, ndc_y, -1.0, 1.0);
        let far_clip = vp_inv * na::Vector4::new(ndc_x, ndc_y, 1.0, 1.0);

        let near_pt = na::Point3::from_homogeneous(near_clip).unwrap_or(self.eye());
        let far_pt = na::Point3::from_homogeneous(far_clip)
            .unwrap_or(na::Point3::new(ndc_x, ndc_y, 1.0));

        let dir = (far_pt - near_pt).normalize();
        (near_pt, dir)
    }

    /// Orbit/pan/zoom handler. Returns true if camera consumed the drag (no picking should happen).
    #[cfg(feature = "gui")]
    pub fn handle_orbit_pan_zoom(&mut self, response: &eframe::egui::Response) {
        // Orbit with left mouse drag (on empty space)
        if response.dragged_by(eframe::egui::PointerButton::Primary) {
            let delta = response.drag_delta();
            self.yaw -= delta.x * 0.005;
            self.pitch += delta.y * 0.005;
            self.pitch = self.pitch.clamp(-1.5, 1.5);
        }
        // Pan with right mouse drag
        if response.dragged_by(eframe::egui::PointerButton::Secondary) {
            let delta = response.drag_delta();
            let right = na::Vector3::new(-self.yaw.sin(), self.yaw.cos(), 0.0);
            let up = na::Vector3::z();
            let pan_speed = self.distance * 0.002;
            self.target -= right * delta.x * pan_speed;
            self.target += up * delta.y * pan_speed;
        }
        // Pan with middle mouse drag (alternative)
        if response.dragged_by(eframe::egui::PointerButton::Middle) {
            let delta = response.drag_delta();
            let right = na::Vector3::new(-self.yaw.sin(), self.yaw.cos(), 0.0);
            let up = na::Vector3::z();
            let pan_speed = self.distance * 0.002;
            self.target -= right * delta.x * pan_speed;
            self.target += up * delta.y * pan_speed;
        }
        // Zoom with scroll
        if response.hovered() {
            let scroll = response.ctx.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.distance *= 1.0 - scroll * 0.002;
                self.distance = self.distance.clamp(0.01, 50.0);
            }
        }
    }

    /// Reset camera to default pose.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Recompute this camera as the third-person trailing view of a
    /// link whose pose is `link_world`. The look-at target is the link
    /// origin shifted by `settings.target_local_offset` (rotated into
    /// world frame); yaw / pitch / distance are taken from `settings`,
    /// with the link's body-yaw added so a `yaw_offset = 0` puts the
    /// camera directly behind the body's local +x axis.
    pub fn update_from_tps(
        &mut self,
        link_world: &na::Isometry3<f32>,
        settings: &TpsSettings,
    ) {
        // Look-at point: link origin + offset rotated into world frame.
        let target_world =
            link_world.translation.vector + link_world.rotation * settings.target_local_offset;
        self.target = na::Point3::from(target_world);

        // Body yaw = atan2(forward.y, forward.x) where forward = R · +x_local.
        let forward_world = link_world.rotation * na::Vector3::x();
        let body_yaw = forward_world.y.atan2(forward_world.x);

        // OrbitCamera convention from `eye()`:
        //   offset = distance · (cos pitch · cos yaw, cos pitch · sin yaw, sin pitch)
        // We want yaw_world = body_yaw + π + yaw_offset so the camera
        // sits BEHIND the body's forward direction. The +π flips us to
        // the opposite side; without it, yaw_offset = 0 would put the
        // camera in front of the body.
        self.yaw = body_yaw + std::f32::consts::PI + settings.yaw_offset;
        self.pitch = settings.pitch_offset;
        self.distance = settings.distance.max(0.05);
    }
}
