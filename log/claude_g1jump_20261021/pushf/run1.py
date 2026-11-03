import json,os,subprocess,sys
src,out=sys.argv[1],sys.argv[2]
env=dict(os.environ); env.update(json.load(open(src))); env.update({'DIAG_PUSHF':'1','DIAG_PUSHF_EVERY':'5'})
env['LD_LIBRARY_PATH']='/home/takara/.mujoco/mujoco-3.8.0/lib'
r=subprocess.run(['log/claude_g1jump_20261021/g1_flight_probe_j33'],env=env,capture_output=True,text=True,cwd='/home/takara/work/dp/articara')
open(out,'w').write(r.stdout+r.stderr)
