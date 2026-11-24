# SENSE_PRED_TICKS (§71)
Latency predictor: misa-locomotion 04ef595 (`RobotModel::predict`), binary tu81. Off = tu80 bit-identical.
- grid_delay_seed1to16.txt / grid_ticks_seed1to16.txt: WR06 nom (48 runs) over SENSE_DELAY_TICKS / ACT_DELAY_TICKS x SENSE_PRED_TICKS.
- battery18.txt: wr05.sh 18 conditions, paired against WR06 (g15kx0 = fresh seeds 17..32, w06a = seeds 1..16).

# SENSE_PRED_AUTO (§74)
Delay identification: misa-locomotion 703b708 + dbfe158 (local, not pushed), binaries tu85 (residual version, dropped), tu86 (dither), tu87 (+ SENSE_PRED_DITHER_T). Off = tu84 bit-identical.
- grid_margin_wr08.txt: WR08 nom over SENSE_DELAY_TICKS x SENSE_PRED_TICKS (pgrid3.sh). The window is L+1..L+3; longer predictions lose.
- auto_delay.txt: wr05.sh 20 conditions paired against WR08 (fixed p6, dither always on with ADD 1/2/3, dither for 1 s with ADD 2/3, fresh seeds), and pgrid4.sh (delays the prediction is not told).
