#!/bin/bash
# eqdiag.sh BIN OUT : eq.sh が見ない診断の出力（デバッグ印字、診断 CSV、MPC／QP のダンプ）を見本 4 本で BIN から取り、OUT/<見本>/ に置く。
# 比較は eqdiagcmp.sh REF NEW（時間の列 *_ns・deadline_miss* とダンプの solve_ns／check_ns を除いて一致を見る）。
# 見本: 段差 2 cm（平地の代わり）、G1 の押し、昇段 4 段、kyo46rs の押し。Walker 切り出しの S0 で作った（doc/from_claude/20261029_walker_extraction_plan.md §5.0）。
# 実行中にこのファイルを編集しないこと（bash が読み進めて壊れる）。
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
BIN=$(readlink -f $1); OUT=$2; mkdir -p $OUT
one() {
  c=log/claude_g1rand_20260924/$1/controls.json; id=$(echo $1 | sed 's#/#__#g'); D=$OUT/$id; rm -rf $D; mkdir -p $D
  ( eval "$(python3 -c "import json,shlex;[print('export %s=%s'%(k,shlex.quote(str(v)))) for k,v in json.load(open('$c')).items()]")"
    export CONTACT_ADMIT_DEBUG=1 DS_FEASIBLE_DEBUG=1 DS_CAPTURE_DEBUG=1 CP_DEBUG=1 PUSH_REPLAN_DEBUG=1 MPC_FS_CROSS_DEBUG=1 \
      MPC_FS_DEBUG_LANDING=1 DEBUG_ANCHOR=1 DIAG_REPLAN=1 MPC_TRAJ_DEBUG=1
    export GAIT_DUMP_T=3,3.2 CONTACT_DUMP_T=3,3.05 WBC_LEVEL_DEBUG_T=3.0
    export TRAJ_CSV=$D/traj.csv PREVIEW_TRAJ_CSV=$D/preview_traj.csv MPC_TRAJ_CSV=$D/mpc_traj.csv MOM_DIAG_CSV=$D/mom_diag.csv \
      TRANSFER_DIAG_CSV=$D/transfer_diag.csv QP_WRENCH_CSV=$D/qp_wrench.csv ANKLE_CBF_DIAG_CSV=$D/ankle_cbf_diag.csv \
      TICK_AUDIT_CSV=$D/tick_audit.csv QP_RUNTIME_CSV=$D/qp_runtime.csv MPC_RUNTIME_CSV=$D/mpc_runtime.csv \
      COM_PREVIEW_RUNTIME_CSV=$D/com_preview_runtime.csv
    export MPC_DIAG_DIR=$D/mpc_diag QP_DIAG_DIR=$D/qp_diag QP_CAPTURE_ALL=1 QP_CAPTURE_START=3 QP_CAPTURE_END=3.05
    $BIN > $D/run.log 2>&1 )
}
export -f one; export BIN OUT
printf '%s\n' FAM_nh10_s/r0.020/m00 FAM_nh10_p/i11_d180/m06 FAM_nh10_asc_n4/r0.060/m00 KV3_push_v2/i0.3_d180/m00 \
  | xargs -P 4 -I{} bash -c 'one {}'
echo "#### EQDIAG_DONE $OUT"
