//! Integration tests for the Rhai scripting engine.

#[cfg(feature = "scripting")]
mod scripting_tests {
    use articara::scripting::{ScriptEngine, ScriptInputs};

    /// Simulate a simple reflex: if foot force exceeds threshold,
    /// apply opposing torque to the knee joint.
    #[test]
    fn reflex_controller() {
        let mut engine = ScriptEngine::new();
        engine
            .compile(
                r#"
            // Reflex: oppose foot force above threshold
            let k = 0.8;
            let threshold = 20.0;

            let f = sensor["foot_force"];
            if f > threshold {
                let tau = -k * (f - threshold);
                set_torque(4, tau);
                debug_val("reflex_active", 1.0);
            } else {
                debug_val("reflex_active", 0.0);
            }
        "#,
            )
            .unwrap();

        // Below threshold — no torque override
        let mut inputs = ScriptInputs::default();
        inputs.sensors.insert("foot_force".to_string(), 10.0);
        inputs.dt = 0.001;
        inputs.time = 0.0;

        let out = engine.eval(&inputs);
        assert!(out.torque_overrides.is_empty());
        assert!((out.debug_values["reflex_active"]).abs() < 1e-9);

        // Above threshold — torque applied
        inputs.sensors.insert("foot_force".to_string(), 45.0);
        inputs.time = 0.001;
        let out = engine.eval(&inputs);
        // tau = -0.8 * (45 - 20) = -20.0
        let tau = out.torque_overrides[&4];
        assert!(
            (tau - (-20.0)).abs() < 1e-9,
            "expected -20.0, got {tau}"
        );
        assert!((out.debug_values["reflex_active"] - 1.0).abs() < 1e-9);
    }

    /// PD controller written in script.
    #[test]
    fn pd_controller_script() {
        let mut engine = ScriptEngine::new();
        engine
            .compile(
                r#"
            let kp = 100.0;
            let kd = 10.0;
            let target = 1.57;   // 90 degrees

            let pos = q["0"];
            let vel = qd["0"];
            let error = target - pos;
            let tau = kp * error - kd * vel;
            set_torque(0, clamp(tau, -50.0, 50.0));
            debug_val("error", error);
        "#,
            )
            .unwrap();

        let mut inputs = ScriptInputs::default();
        inputs.joint_positions.insert(0, 0.5);
        inputs.joint_velocities.insert(0, 0.1);
        inputs.dt = 0.001;

        let out = engine.eval(&inputs);
        // error = 1.57 - 0.5 = 1.07
        // tau = 100*1.07 - 10*0.1 = 107 - 1 = 106 → clamped to 50
        assert!(
            (out.torque_overrides[&0] - 50.0).abs() < 1e-9,
            "expected clamped 50.0, got {}",
            out.torque_overrides[&0]
        );
    }

    /// State accumulation across multiple eval cycles (e.g. integral term).
    #[test]
    fn integral_accumulation() {
        let mut engine = ScriptEngine::new();

        // First compile sets up the integrator variable
        engine.compile(r#"let integral = 0.0;"#).unwrap();
        engine.eval(&ScriptInputs::default());

        // Second compile uses the persistent variable
        engine
            .compile(
                r#"
            let error = 1.0;
            integral += error * dt;
            debug_val("integral", integral);
        "#,
            )
            .unwrap();

        let mut inputs = ScriptInputs::default();
        inputs.dt = 0.01;

        // Run 10 cycles
        let mut last_integral = 0.0;
        for i in 0..10 {
            inputs.time = (i as f64) * inputs.dt;
            let out = engine.eval(&inputs);
            last_integral = out.debug_values["integral"];
        }

        // integral should be approximately 10 * 1.0 * 0.01 = 0.10
        assert!(
            (last_integral - 0.10).abs() < 1e-9,
            "expected ~0.10, got {last_integral}"
        );
    }

    /// Verify that scripts cannot run forever.
    #[test]
    fn runaway_protection() {
        let mut engine = ScriptEngine::new();
        engine.compile("let x = 0; while true { x += 1; }").unwrap();

        let start = std::time::Instant::now();
        let _out = engine.eval(&ScriptInputs::default());
        let elapsed = start.elapsed();

        // Should terminate quickly (< 1s) via max_operations
        assert!(
            elapsed.as_secs() < 1,
            "runaway script took too long: {elapsed:?}"
        );
        assert!(engine.last_error().is_some());
    }
}
