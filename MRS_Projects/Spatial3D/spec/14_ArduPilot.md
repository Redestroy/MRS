# 14 The ArduPilot platform

Status: **WP10**, spec version `0.1`. Plan reference: §4.4 (ArduPilot platform), §5.7 (drivers behind the nodes), §6.3 (pass-through processors), §11 (WP10). Builds on specs [02](02_Actions.md) (flight actions), [04](04_Devices_and_Ports.md) (devices and ports), [05](05_Worldview.md) (worldview), [08](08_Robot_and_Flight.md) (robot and flight) and [09](09_MRS_Layer.md) (MRS layer).

## 1. Scope

On ArduPilot, the flight controller stabilises the airframe and fuses its own sensors (EKF). The library does not drive the motors. It tells the autopilot where to go, in GUIDED mode, and reads the autopilot's estimates instead of raw sensors. Everything above the device tree stays as it is: the same task strings, behaviours, robot controller, MRS layer and allocators as on Webots and QuadSim.

WP10 adds:

* **`ArduPilotPlatform`**: an `IPlatform` whose ports are MAVLink channels (§3);
* **`GuidedFcu`**: the flight control unit as GUIDED-mode commands (§4);
* **pass-through processing**: the autopilot's position, velocity and attitude feed the worldview unchanged (§5);
* **`UdpTransport`**: messages between agents over UDP (§6);
* **`MockAutopilot`**: an in-process stand-in for ArduCopter, for tests and CI (§7);
* the programs **`mrs_ardupilot_uav`** and **`mrs_operator`** (§8);
* runs against **ArduPilot SITL** (§9).

Code: `platforms/ardupilot/` (library `mrs_ardupilot`). MAVLink's generated C headers ([c_library_v2](https://github.com/mavlink/c_library_v2), common dialect, MAVLink 2, MIT licence) are the only new dependency. They are vendored in `platforms/ardupilot/third_party/mavlink`, so the library builds without network access (`MRS_MAVLINK_DIR` can point at another copy). Only the sources in `platforms/ardupilot/src` include them.

## 2. Links

`Net::OpenLink(url)` opens the byte link to the autopilot:

| URL | Link | Use |
|---|---|---|
| `tcp:HOST:PORT` | TCP client; it reconnects once a second when the link drops | SITL instance *i* listens on `tcp:127.0.0.1:5760 + 10 i` |
| `udpin:PORT` | UDP, listening; it answers the last sender | SITL `--out`, MAVProxy, a telemetry radio bridge |
| `udp:HOST:PORT` | UDP, sending to HOST:PORT | a flight controller's network port |

A serial port is not in version 0.1. On hardware, a companion computer bridges the flight controller's serial port to UDP (for example `mavlink-routerd`).

## 3. `ArduPilotPlatform`

### 3.1 The connection

The platform is a MAVLink component with system id `system_id` (default 245; `mrs_ardupilot_uav` uses 245 − robot id) and component `MAV_COMP_ID_ONBOARD_COMPUTER`. It sends a `HEARTBEAT` every second. Its target is the first autopilot whose heartbeat it hears, unless `target_system` names one. Every 10 s it asks the target, with `MAV_CMD_SET_MESSAGE_INTERVAL`, for `GLOBAL_POSITION_INT`, `ATTITUDE` and `LOCAL_POSITION_NED` at `stream_hz` (10 Hz), and for `BATTERY_STATUS`, `SYS_STATUS` and `EXTENDED_SYS_STATE` at 2 Hz.

`WaitForAutopilot(s)` reads the link until the autopilot's heartbeat and a position fix arrive. A position of exactly 0, 0 means the EKF has no origin yet.

### 3.2 The clock

| `clock` | Mission time | Use |
|---|---|---|
| `WALL` (default) | seconds since `epoch` (unix s) on the computer's clock; with no `epoch`, the last UTC midnight | a team: every agent must read the same clock, because message stamps are compared across agents (spec 06 §5) |
| `AUTOPILOT` | the autopilot's boot time | one robot on SITL with `--speedup` |
| `MANUAL` | one `period` per `Step` | tests, with `MockAutopilot` |

`Step` reads the link until mission time has advanced by `period` (0.02 s). With `WALL`, a tick that comes late catches up rather than bursting.

On hardware, the agents' clocks must agree to well under the peer timeout (spec 06 §5), for example through NTP or GNSS time.

### 3.3 Ports

All ports have type `MAVLINK`. They are addressed by channel name:

| Address | Read | Write |
|---|---|---|
| `GLOBAL_POSITION_INT` | lat, lon (deg), alt AMSL (m) | |
| `VELOCITY` | vx, vy, vz: ENU m/s (from `GLOBAL_POSITION_INT`) | |
| `ATTITUDE` | roll, pitch, yaw in the library's frames (spec 00 §3): roll, −pitch, π/2 − yaw | |
| `ATTITUDE_RATES` | p, −q, −r (FRD to FLU) | |
| `BATTERY_STATUS` | −1, voltage V, remaining 0..1. `SYS_STATUS` supplies these until a `BATTERY_STATUS` arrives | |
| `GUIDED` | alive, armed, custom mode, landed state, local OK, local east, north, up, ack count, last ack command, last ack result | a command (below) |
| `RELAY:n` | | on (non-zero) or off: `MAV_CMD_DO_SET_RELAY` n. Only changes are sent |
| `SERVO:n` | | refused. The node is there for the self model |

Sampled ports return a value once per new message. A `GUIDED` write is a command code, followed by its values:

| Code | Command | Values | MAVLink |
|---|---|---|---|
| 1 | `SET_MODE` | custom mode (ArduCopter: 4 GUIDED, 9 LAND, 6 RTL) | `MAV_CMD_DO_SET_MODE` |
| 2 | `ARM` | 1 arm, 0 disarm | `MAV_CMD_COMPONENT_ARM_DISARM` |
| 3 | `TAKEOFF` | altitude above home, m | `MAV_CMD_NAV_TAKEOFF` |
| 4 | `POSITION` | north, east, down (local NED, m), yaw (rad, from North) | `SET_POSITION_TARGET_LOCAL_NED`, position and yaw |
| 5 | `VELOCITY` | vn, ve, vd (m/s), yaw rate (rad/s, NED) | `SET_POSITION_TARGET_LOCAL_NED`, velocity and yaw rate |
| 6 | `SPEED` | horizontal speed for position targets, m/s | `MAV_CMD_DO_CHANGE_SPEED` |

`Scan` lists every address above (relays 0 to 5, servos 1 to 8), with identity `ardupilot` once a heartbeat has arrived.

## 4. The flight control unit and the device keys

### 4.1 `fcu.mavlink`: `GuidedFcu`

`GuidedFcu` derives from `FlightControlUnit`: it has the same actions (`A_TO`, `A_LD`, `A_HD`, `A_PXY`, `A_PZY`, `A_VXY`, `A_VZY`, spec 02), the same gains for limits (`max_speed_xy`, `max_climb`, `max_yaw_rate`, `kp_pos`, `kp_yaw`, `takeoff_tol`, `setpoint_timeout`) and the same modes. The robot controller and the safety supervisor see no difference. It writes only to its `guided` port and drives no motor.

| Action | GUIDED commands | Ends |
|---|---|---|
| `A_TO agl rate` on the ground | Once a second: `SET_MODE` GUIDED until the autopilot reports it, then `ARM`. Once armed: `SPEED max_speed_xy`, then `TAKEOFF agl`, again after 3 s while still below 0.3 m | DONE at `alt.agl ≥ agl − takeoff_tol`. FAILED when not armed within `arm_timeout` (60 s) |
| `A_TO` in the air | a position target at the new height | as above |
| `A_LD` | `SET_MODE` LAND, once a second until the autopilot reports it | the autopilot disarms: the mode becomes DISARMED, and `A_LD` is DONE |
| `A_HD`, `A_PXY`, `A_PZY`, `A_VXY`, `A_VZY` | the setpoints of spec 08 §2, kept as in the library's fcu | at once, as there |

In FLY, the unit sends one target every 0.1 s.

* **Position on both axes**: a `POSITION` target.
* **Velocity on either axis**: ArduPilot takes a target of only one kind, so each position axis becomes a velocity, `kp_pos × error`, limited by `max_speed_xy` and `max_climb`. The result is sent as a `VELOCITY` target. Velocity setpoints that are not renewed within `setpoint_timeout` become a position hold, as in spec 08 §2.3.

**Frames.** Targets are in the mission ENU frame (spec 00 §3). The autopilot's local frame starts at its own EKF origin, usually where it booted. The unit takes the offset between the two from the same estimate: the mission-frame flight state (from `GLOBAL_POSITION_INT` through the geo reference) minus the local position from `LOCAL_POSITION_NED`. Version 0.1 sends no target until both are known.

**Faults.** When the autopilot is silent for more than 2 s, `InFailsafe` is true, and the safety layer acts as it does for a lost flight state (spec 08 §6). A disarm by the autopilot itself in flight (a crash, or its own failsafe) ends the flight: the mode becomes DISARMED. The autopilot's own failsafes (battery, GCS loss, geofence) stay in charge. The library does not fight a mode the autopilot changes on its own.

### 4.2 Device keys

`RegisterArduPilotDevices` adds:

| Key | Class | Defaults |
|---|---|---|
| `quadrotor.mavlink` | `Quadrotor` (spec 04 §3) | `port MAVLINK`. Motors `SERVO:3`, `SERVO:1`, `SERVO:2`, `SERVO:4`: ArduCopter's quad X numbering of front left, front right, rear left and rear right. IMU `ATTITUDE`, `ATTITUDE_RATES`. Its fcu is `fcu.mavlink` |
| `motor.mavlink`, `imu.mavlink` | `RotorMotor`, `Imu` | `port MAVLINK` |
| `fcu.mavlink` | `GuidedFcu` | `guided "GUIDED"`; parameter `arm_timeout` (s) |
| `gnss.mavlink` | `Gnss` | `device "GLOBAL_POSITION_INT"`; `V_GEO` |
| `vel.mavlink` | `Velocity` | `device "VELOCITY"`; `V_VEL3` |
| `led.mavlink` | `LedArray` | `port MAVLINK`; `device "RELAY:n"` per LED |
| `battery.mavlink` | `Battery` | `device "BATTERY_STATUS"`; the port's remaining fraction × `capacity_wh` is the energy |

`Quadrotor::Expand` now uses `fcu.<platform>` when the registry has that key, and `fcu.default` otherwise (amends spec 04 §3). `Velocity` is a new core sensor: `V_VEL3` from a port that reads ENU velocity.

[`examples/quad_ardupilot.mrsd`](examples/quad_ardupilot.mrsd) and its port map [`quad_ardupilot.mrsp`](examples/quad_ardupilot.mrsp) describe an ArduCopter quad with GNSS, velocity, two LEDs on relays 0 and 1, and a 50 Wh battery.

## 5. Pass-through processing (amends spec 05 §5)

* `VelocityProcessor` subscribes to `V_VEL3` and offers `vel.enu` with source `nav`. `KinematicsEstimator` now *offers* `vel.enu` with source `kinematics`, instead of providing it. The source order of `vel.enu` is `nav`, `kinematics`. Without a `V_VEL3`, nothing changes.
* `GLOBAL_POSITION_INT` is the robot's `V_GEO`: `pose.enu` comes from `GeoToLocalProcessor` (source `gnss`) and `alt.amsl` from GNSS, as on Webots.
* `alt.agl` comes from `alt.amsl` minus the home altitude (spec 05 §5.1). The mission header's home for the robot must therefore be where the copter stands. On SITL this is the `--home` the script gives each instance (§9).
* `ATTITUDE` is `V_ATT` and `V_RATE`; `heading` comes from the attitude.

## 6. `UdpTransport`

`UdpTransport` is an `ITransport` (spec 06 §2) with broadcast semantics, like the Webots radio. Each agent listens on a UDP port and sends every message to each of its `peers`. With `multicast` set, every agent joins the group, listens on the same port and sends to the group. One message is one datagram, at most `max_payload` (60000) bytes. A larger message is refused, as on any transport (spec 06 §2).

On one computer, each agent has its own port. The defaults of the programs in §8 are: operator 14600, robot *k* 14600 + *k*, and every agent sends to all of 14600 to 14600 + N. A robot receives its own messages back, and the messenger drops them (spec 06 §2).

## 7. `MockAutopilot`

The mock autopilot is an in-process stand-in for ArduCopter. It speaks real MAVLink bytes over a loopback link, so the platform, the codec and `GuidedFcu` are tested end to end in CI, where SITL cannot run.

**What it accepts:**

* `DO_SET_MODE`: modes 0, 4, 5, 6 and 9;
* `COMPONENT_ARM_DISARM`: arms only in GUIDED, and only `ready_after` (2 s) after start;
* `NAV_TAKEOFF`: when armed, in GUIDED and on the ground;
* `DO_CHANGE_SPEED` and `SET_MESSAGE_INTERVAL`;
* `SET_POSITION_TARGET_LOCAL_NED`: position with yaw, or velocity with yaw rate.

**How it flies:** the airframe is kinematic. Velocity follows the target with a 0.4 s lag. It climbs at up to 2.5 m/s and descends at 1 m/s in LAND. It disarms 1 s after touchdown in LAND, or 10 s after arming without a takeoff.

**What it sends:**

* `HEARTBEAT`, `BATTERY_STATUS` and `EXTENDED_SYS_STATE` at 1 Hz;
* `GLOBAL_POSITION_INT`, `ATTITUDE` and `LOCAL_POSITION_NED` at 10 Hz;
* a `COMMAND_ACK` for every command.

It is not a model of ArduPilot's controllers. Flight times on it say nothing about ArduPilot's; SITL does (§9).

## 8. Programs

`mrs_ardupilot_uav <robot id>` is `mrs_uav` (spec 09) on ArduPilot:

* it connects to the autopilot (`--link`, default `tcp:127.0.0.1:` 5760 + 10 (id − 1), SITL instance id − 1) and waits for its heartbeat and a fix;
* it builds the robot from `--def` and `--ports` (default `quad_ardupilot.mrsd`, `.mrsp`), with head id = robot id;
* it runs the robot controller and the MRS layer with `--alloc` (`rta`, `rta-x`, `sta`, `cbba`, `ldta2`) over `UdpTransport` (`--port`, `--peers`, `--team`, `--multicast`).

`mrs_operator <mission header> <timeline>` is `mrs_issuer` (spec 09 §6) on UDP:

* the timeline's time 0 is `--delay` (5 s) after it starts;
* it writes `results.csv` when every task has ended, or after `--limit` s.

**Clock.** Both programs take `--epoch UNIX_S`, and every agent of a team needs the same value (§3.2).

## 9. SITL

The acceptance of plan §11 WP10 has three steps:

1. the same task strings and the same allocator against ArduPilot SITL alone;
2. then SITL with Webots;
3. then one real UAV flying Liftoff, FlyTo and Land.

Version 0.1 does step 1. Steps 2 and 3 need JB's machines (§10).

**SITL alone.** ArduCopter 4.6.3 (`Copter-4.6.3`), built with `./waf configure --board sitl && ./waf copter`. Each instance *i* runs `arducopter --model + --speedup 1 --defaults Tools/autotest/default_params/copter.parm --sysid i+1 -I i --home LAT,LON,ALT,0`, at the geodetic position of robot *i* + 1's home in the mission header. `experiments/uav_ardupilot/run_sitl.sh` starts N instances, N robots and the operator, and stops them all at the end.

RESULTS

## 10. Not in version 0.1

* **Webots with SITL** (acceptance step 2). ArduPilot ships a Webots integration (`libraries/SITL/examples/Webots_Python`, SITL model `webots-python`), in which Webots supplies the physics and sensors and SITL flies. `mrs_ardupilot_uav` connects to that SITL exactly as in §9; nothing in the library changes. It needs a Webots version the integration supports, and SITL on Linux or WSL.
* **A real flight** (acceptance step 3). One copter flies `uav_single_flight.mrst` (take off to 5 m, fly to a point, land) with `mrs_ardupilot_uav` on a companion computer or a laptop on the telemetry link. It needs a safety pilot with an RC override, the geofence of the mission header inside a legal flying area, and the autopilot's own failsafes configured.
* A serial link, MAVLink signing, and more than one autopilot per link.
* Rangefinder and relative-altitude pass-through. `alt.agl` assumes flat ground at home (§5).

## 11. Tests (WP10)

`tests/ardupilot/test_ardupilot.cpp` (spec 07 §5.12), on `MockAutopilot` with the `MANUAL` clock:

* the platform finds the autopilot, and its ports read the estimate in the library's frames: the geodetic position of the home, ENU yaw π/2 for NED yaw 0, the GUIDED status; a servo port refuses writes;
* one copter flies `uav_single_flight.mrst` (take off, fly to (40, 20, 10), flash the LEDs, land) from the same task strings as on QuadSim. The autopilot's origin is 10 m east and 5 m north of the mission's, and the copter still ends within 1.5 m of the target. It used GUIDED targets, kept to `max_speed_xy`, and is disarmed at the end;
* a takeoff that cannot arm fails after `arm_timeout`, with the autopilot left in GUIDED and disarmed;
* two copters and the task issuer share four point tasks through the MRS layer with MRS-RTA-X: every task is done once, and both copters work;
* `UdpTransport` delivers a message over the loopback interface and refuses one larger than a datagram;
* link URLs and endpoints parse; an unknown link kind throws.
