// March-in-place verification script.
//
// Drives namiashi through:
//   1. constrain pose → mj_start → 1 s settle
//   2. gait_setup + champ + start
//   3. gait_set_velocity(1e-6, 0, 0)   ← the magic ε used by the 👣 button
//   4. 4 s walk (= 10 trot cycles)
//   5. Save CSV
//
// If march-in-place works, the foot z trajectories should show 10 swing
// peaks each leg lifting up to swing_height (≈ 4 cm), while body_px stays
// within < 1 mm of its post-settle value.

set_joint("FL_hip_joint",    0.0);
set_joint("FL_thigh_joint",  1.0);
set_joint("FL_calf_joint",  -2.0);
set_joint("FR_hip_joint",    0.0);
set_joint("FR_thigh_joint",  1.0);
set_joint("FR_calf_joint",  -2.0);
set_joint("RL_hip_joint",    0.0);
set_joint("RL_thigh_joint",  1.0);
set_joint("RL_calf_joint",  -2.0);
set_joint("RR_hip_joint",    0.0);
set_joint("RR_thigh_joint",  1.0);
set_joint("RR_calf_joint",  -2.0);
set_joint("arm_pitch_joint", 0.0);

set_ground_plane_enabled(true);
if mj_active() { mj_stop(); }
mj_start();
mj_set_trace_max(40000);

mj_async_clear();
mj_async_print("[march-test] settle 1 s");
mj_async_step_seconds(1.0);

gait_setup();
set_gait_mode("champ");
gait_start();
mj_async_print("[march-test] gait started, cmd=0, 0.5 s stance");
mj_async_step_seconds(0.5);

// ε command — exactly what the 👣 button sends.
mj_async_set_velocity(1e-6, 0.0, 0.0);
mj_async_print("[march-test] cmd=(1e-6, 0, 0), 4 s walk");
mj_async_step_seconds(4.0);

mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(0.5);
mj_async_save_csv("/tmp/march_test.csv");
mj_async_print("[march-test] DONE — CSV: /tmp/march_test.csv");
