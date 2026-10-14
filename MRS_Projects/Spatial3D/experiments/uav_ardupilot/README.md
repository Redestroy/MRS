# ArduPilot SITL experiments (WP10, spec 14 §9)

The same mission header, task strings and allocators as `../uav_spatial`, flown by ArduCopter SITL
instead of QuadSim or Webots. Each robot is an `mrs_ardupilot_uav` process talking MAVLink to its own
SITL instance; the robots and `mrs_operator` talk over UDP on localhost.

## Setup (Linux or WSL)

```
git clone --recurse-submodules -b Copter-4.6.3 https://github.com/ArduPilot/ardupilot.git
cd ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y   # or pip install empy==3.3.4 pexpect future
./waf configure --board sitl && ./waf copter
```

Build libmrs as usual (spec 07); `MRS_BUILD_ARDUPILOT` is on by default and builds
`platforms/ardupilot/mrs_ardupilot_uav` and `mrs_operator`.

## Running

```
export ARDUPILOT=/path/to/ardupilot MRS_BUILD=/path/to/libmrs/build
./run_sitl.sh 3 ../uav_spatial/tasksets_2021/taskset1.mrsl rta results/my_run
python3 summarize.py results results/wp10/quadsim_taskset1.csv
```

`run_sitl.sh N TIMELINE ALLOC OUTDIR` starts N SITL copters at the homes of robots 1..N in
`mission_5uav.mrs` (`home.py` converts them to latitude and longitude), N robots with allocator
ALLOC (rta, rta-x, sta, cbba, ldta2) and the operator, which starts the timeline 45 s later (pre-arm
checks). It writes `results.csv`, `op.log` and `r<i>.log` to OUTDIR and stops every process at the end.
SITL runs in real time, so a run takes as long as the mission. `sitl.parm` gives SITL a 20 Ah pack.

`single_flight.mrsl` is acceptance step 3 (take off, fly, land) for one copter; fly it in SITL with
`run_sitl.sh 1 single_flight.mrsl rta OUTDIR` before a real flight (spec 14 §10).

Results: `results/wp10/README.md`.
