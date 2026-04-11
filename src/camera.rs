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

    pub fn handle_response(&mut self, response: &eframe::egui::Response) {
        // Orbit with left mouse drag
        if response.dragged_by(eframe::egui::PointerButton::Primary) {
            let delta = response.drag_delta();
            self.yaw -= delta.x * 0.005;
            self.pitch += delta.y * 0.005;
            self.pitch = self.pitch.clamp(-1.5, 1.5);
        }
        // Pan with middle or right mouse drag
        if response.dragged_by(eframe::egui::PointerButton::Middle)
            || response.dragged_by(eframe::egui::PointerButton::Secondary)
        {
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
}
