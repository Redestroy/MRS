#!/bin/bash
# run_sitl.sh N TIMELINE ALLOC OUTDIR
# N ArduCopter SITL instances, N mrs_ardupilot_uav robots and one mrs_operator on this machine (spec 14 §9).
# Needs ARDUPILOT (an ArduPilot checkout with build/sitl/bin/arducopter built by `./waf copter`)
# and MRS_BUILD (a libmrs build directory). Kills everything it started when it ends.
set -u
N=$1; TL=$2; ALLOC=$3; OUT=$4
: "${ARDUPILOT:?set ARDUPILOT to the ArduPilot checkout}"
: "${MRS_BUILD:?set MRS_BUILD to the libmrs build directory}"
HERE=$(cd "$(dirname "$0")" && pwd)
MISSION=${MISSION:-$HERE/../uav_spatial/mission_5uav.mrs}
B=$MRS_BUILD/platforms/ardupilot
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
cleanup() { kill $(jobs -p) 2>/dev/null; pkill -P $$ 2>/dev/null; }
trap cleanup EXIT
# Start points in mission ENU (the same as the QuadSim and Webots runs of mission_5uav).
homes=("-8 -5 0" "-4 -5 0" "0 -5 0" "4 -5 0" "8 -5 0")
(( N >= 1 && N <= 5 )) || { echo "N must be 1..5 (the robots of mission_5uav)"; exit 2; }
for ((i=0;i<N;i++)); do
  d=$OUT/sitl$i; mkdir -p "$d"
  H=$(python3 "$HERE/home.py" ${homes[$i]})
  (cd "$d" && exec "$ARDUPILOT/build/sitl/bin/arducopter" --model + --speedup 1 \
     --defaults "$ARDUPILOT/Tools/autotest/default_params/copter.parm,$HERE/sitl.parm" \
     --sysid $((i+1)) -I$i --home "$H" -w > sitl.log 2>&1) &
done
sleep 2
EP=$(date +%s)   # shared mission epoch for every agent
for ((i=1;i<=N;i++)); do
  (cd "$OUT" && exec "$B/mrs_ardupilot_uav" $i --team $N --alloc "$ALLOC" --epoch $EP > r$i.log 2>&1) &
done
# 45 s lets every copter pass its pre-arm checks before the first task.
(cd "$OUT" && "$B/mrs_operator" "$MISSION" "$TL" --team $N --epoch $EP --results results.csv --delay 45 --limit 1500 > op.log 2>&1)
echo "operator exit $?"
tail -1 "$OUT/op.log"
