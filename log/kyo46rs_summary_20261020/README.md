# kyo46rs まとめ動画（2026-10-20）
`kyo46rs_summary_20261020.mp4`（72 s）。場面ごとの controls は各ディレクトリ、バイナリは `log/claude_g1rand_20260925/kyo46rs_walk_tu31`。
再現: 各場面を `TRAJ_CSV=traj.csv PREVIEW_TRAJ_CSV=preview.csv` 付きで走らせ、`render_scene.sh NAME T0 T1 TITLE SUBTITLE`（区間は 1: 3–11、2: 3–9、3: 6.9–11、4/5: 7.5–11.5、6: 13.8–19.5、7: 13.8–19.8 s）。
重ね描き: `examples/render_com_squat_video.py` の `TARGETS=1`（3D 上と右下の平面図）。予見 MPC（`COM_REPLAN`）は段の着地時刻の管理が始まってから起動するので、平地と押しの場面では DCM 計画の参照だけが描かれる。kyo46rs の基準は SRBD レンチ MPC を使っていない（`MPC_CSV` は G1 用）。
