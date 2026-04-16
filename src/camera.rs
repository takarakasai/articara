use nalgebra as na;

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
}
