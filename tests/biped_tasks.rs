
/// The trunk orientation error must be expressed in the world frame at ANY
/// heading. Feeding a ZYX roll/pitch error straight into the world angular
/// rows is only the same thing at yaw=0; a quarter turn later the body's roll
/// axis points along world +y, so the correction lands on the wrong axis, and
/// past 90 deg it is wrong-signed too. That bug made every kyo46rs turn
/// command fall after a fixed ACCUMULATED heading (120 deg at wz=0.10, 104 at
/// 0.20, 90 at 0.40) while tracking its rate to ~90% right up to the fall.
#[test]
fn trunk_orientation_error_is_frame_correct_at_any_heading() {
    use articara::biped::tasks::{trunk_ori_ref, TrunkGains};
    use nalgebra as na;

    let gains = TrunkGains {
        kp: 100.0,
        kd: 0.0,
        deadband: 0.0,
        sign: 1.0,
        kp_yaw: 0.0,
        kd_yaw: 0.0,
        wz_ref: 0.0,
    };
    let tilt = 0.05_f64;
    // A body tilted about its OWN roll axis, at four headings. The correction
    // must always be the same magnitude and always oppose the tilt -- what
    // rotates is which world axis carries it.
    for (yaw, want) in [
        (0.0, [-1.0, 0.0]),
        (std::f64::consts::FRAC_PI_2, [0.0, -1.0]),
        (std::f64::consts::PI, [1.0, 0.0]),
        (-std::f64::consts::FRAC_PI_2, [0.0, 1.0]),
    ] {
        let r = na::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw)
            * na::UnitQuaternion::from_euler_angles(tilt, 0.0, 0.0);
        let a = trunk_ori_ref(&r, Some(yaw), &[0.0; 3], gains);
        let mag = gains.kp * tilt;
        assert!(
            (a[0] - want[0] * mag).abs() < 1e-9 && (a[1] - want[1] * mag).abs() < 1e-9,
            "yaw={yaw:.3}: got [{:.4}, {:.4}], want [{:.4}, {:.4}]",
            a[0], a[1], want[0] * mag, want[1] * mag
        );
        assert!(a[2].abs() < 1e-9, "yaw={yaw:.3}: heading is on target, z must be 0");
    }

    // And the yaw row must not wrap: a target 10 deg PAST a half turn is a
    // small positive correction, not a large negative one.
    let g = TrunkGains { kp: 0.0, kp_yaw: 1.0, ..gains };
    let r = na::UnitQuaternion::from_euler_angles(0.0, 0.0, 3.10);
    let a = trunk_ori_ref(&r, Some(-3.10), &[0.0; 3], g);
    let expect = std::f64::consts::TAU - 6.20;
    assert!((a[2] - expect).abs() < 1e-9, "got {:.4}, want {:.4}", a[2], expect);
}
