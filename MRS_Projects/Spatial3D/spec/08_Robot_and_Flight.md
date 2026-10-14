# 08 Robot layer and flight

Status: **WP4**, spec version `0.1`. Plan reference: §5.7 (generic flight machine), §7 (robot layer), §11 (WP4). Builds on specs [02](02_Actions.md) (flight actions), [03](03_Tasks_and_Execution.md) (executor), [04](04_Devices_and_Ports.md) (devices), [05](05_Worldview.md) (worldview) and [06](06_Messages.md) §8 (journal).

## 1. Scope

WP4 makes one UAV fly its tasks:

* the **flight control unit** (`fcu`) turns the flight actions of spec 02 §4.2 into motor speeds (§2);
* the **robot controller** runs the agent loop (§3) and handles events (§4);
* the **safety supervisor** filters actions and starts safety behaviours (§5);
* the **resource manager** tracks energy and estimates task cost (§6);
* the **task journal** lets a robot resume after a battery swap (§7);
* the **UAV behaviour library** gains the remaining fly-to variants and an emergency landing (§8).

The MRS layer (allocation, messages) is WP5. In WP4 the robot works through a local task list in file order.

## 2. Flight control unit

### 2.1 State it reads

The `fcu` is an actuator, so it does not own the worldview. The robot controller gives it a **state source**, a function returning the current flight state from the worldview, or nothing when a field is missing or stale:

| State | Worldview field |
|---|---|
| position `p` | `pose.enu` |
| velocity `v` | `vel.enu` |
| attitude roll, pitch, yaw | `att` |
| body rates `ω` | `rate.body` |
| height above ground | `alt.agl` |

The controller writes the `fcu`'s `armed` state into the worldview as the predicate `armed` (source `fcu`) every tick, before the worldview update, so `airborne` sees the current state.

### 2.2 Modes

| Mode | Entered by | Reports |
|---|---|---|
| `DISARMED` | start; the end of a landing | motors 0 |
| `TAKEOFF` | `A_TO h r` (arms if needed) | `RUNNING` until `|alt.agl − h| ≤ 0.3 m`, then `DONE` and `HOLD` there |
| `FLY` | a position or velocity setpoint, or `A_HD` | setpoints `DONE` at once; `A_HD d` with `d > 0` `RUNNING` until `d` has passed |
| `LANDING` | `A_LD r` | `RUNNING` until `alt.agl < 0.15 m` and `|v| < 0.2 m/s` for 0.5 s, then disarms and reports `DONE` |

* While `DISARMED`, `A_LD` and `A_HD` report `DONE` at once (the robot is down and still), so a landing behaviour that loops sees its landing end. Every other action except `A_TO` is `REJECTED`.
* In `FLY`, horizontal, vertical and yaw control each keep their own mode. `A_PXY` sets horizontal position, `A_VXY` horizontal velocity; `A_PZY` sets vertical position and yaw angle, `A_VZY` vertical velocity and yaw rate. A missing half keeps its last value (spec 02 §4.2).
* A velocity setpoint not renewed within `setpoint_timeout` (0.5 s) is replaced by a position hold at the current position.
* With no flight state while armed, the `fcu` keeps its last commands for `state_timeout` (0.25 s), which covers the first ticks after arming and a late sample. After that it levels the airframe and descends with thrust slightly below hover (`failsafe`), and the controller raises a FAULT event once (§4).

### 2.3 Cascade

Every control tick (`Update`, after dispatch), from outside in:

1. **Position → velocity.** `v_cmd = Kp_pos · (p_cmd − p)`, horizontal magnitude clamped to `max_speed_xy`, vertical to `max_climb` (or the commanded climb or descent rate in `TAKEOFF` and `LANDING`). A velocity setpoint replaces this step for its axis.
2. **Velocity → acceleration.** PI per axis: `a_cmd = Kp_vel · (v_cmd − v) + Ki_vel · ∫(v_cmd − v)`. The horizontal magnitude is clamped to `g · tan(max_tilt)`; the integrators are clamped against wind-up.
3. **Acceleration → attitude.** In the yaw frame, forward acceleration `a_f` and leftward `a_l`: `pitch_cmd = a_f / g`, `roll_cmd = −a_l / g`, both clamped to `max_tilt`. The vertical acceleration becomes the vertical input `u_z = Kv · a_z + I_z`, where the integrator `I_z` learns the hover thrust offset.
4. **Yaw.** `r_cmd = Kp_yaw · wrap(yaw_cmd − yaw)` clamped to `max_yaw_rate`, or the commanded yaw rate; `u_yaw = Kr · (r_cmd − r)`.
5. **Attitude → torque inputs.** `u_roll = Kp_att · (roll − roll_cmd) + Kd_att · p` and `u_pitch = Kp_att · (pitch − pitch_cmd) + Kd_att · q`, each clamped.
6. **Mixer** (the Webots Mavic layout, as in the Cyberbotics example):

| Motor | Speed command (rad/s) |
|---|---|
| `m_fl` | `+(ω_h + u_z − u_roll + u_pitch − u_yaw)` |
| `m_fr` | `−(ω_h + u_z + u_roll + u_pitch + u_yaw)` |
| `m_rl` | `−(ω_h + u_z − u_roll − u_pitch + u_yaw)` |
| `m_rr` | `+(ω_h + u_z + u_roll − u_pitch − u_yaw)` |

The four commands are one `A_MAP` of `A_MOT` entries applied to the motor nodes (spec 02 §5). Each magnitude is clamped to `[0, max_motor]`.

### 2.4 Parameters

`fcu.default` parameters, with defaults for the Webots Mavic 2 Pro. A `quadrotor.*` line passes these keys on to its `fcu`.

| Key | Default | Key | Default |
|---|---|---|---|
| `hover_speed` (`ω_h`) | 68.5 rad/s | `max_motor` | 576 rad/s |
| `max_speed_xy` | 8 m/s | `max_climb` | 2 m/s |
| `max_tilt` | 0.35 rad | `max_yaw_rate` | 1.0 rad/s |
| `kp_pos` | 0.9 | `kp_vel`, `ki_vel` | 1.6, 0.5 |
| `kv` | 4.0 | `ki_z` | 6.0 |
| `kp_yaw`, `kr` | 1.5, 6.0 | `kp_att`, `kd_att` | 50, 4 |
| `setpoint_timeout` | 0.5 s | `takeoff_tol` | 0.3 m |
| `state_timeout` | 0.25 s | | |

The gains were tuned on the test simulator (§9), whose motor constants come from the proto but whose mass, inertia and power are estimates. With these gains the simulator takes off, flies 45 m and lands without a crash for masses from 0.4 to 1.1 kg; the hover integrator `ki_z` absorbs the difference to the mass `hover_speed` implies. In Webots the default `kp_att` 50 is too stiff: the Mavic oscillates in roll and pitch about 2 s after liftoff and flips. With `kp_att 20` (set on the `quadrotor` line of `mavic_webots.mrsd`) and the stock Mavic world's `defaultDamping` (linear and angular 0.5 in `WorldInfo`), the three-UAV team world flies all 15 tasks of task set 1. Without that damping the robots stay up but stall on their tasks. Values between 5 and 20 were stable in Webots; 50 flipped with any `kd_att` tried (1 to 8).

## 3. Robot controller

`RobotController` lives with the head node and owns, for one robot: the device side (`Device::Robot`, spec 04), the `WorldModel` (spec 05), the behaviour library, the `TaskExecutor`, the safety supervisor, the resource manager, the journal and the local task list.

Life cycle hooks, in the order they run: `OnInit` and `OnStart` (once, at the start of the first tick), `OnPreUpdate`, `OnAct`, `OnPostUpdate` (every tick), `OnStop` (when task execution stops). A subclass overrides them; the default does nothing.

### 3.1 One tick

`Tick(t)`, non-blocking; the platform owns the outer loop:

1. Write `armed` from the `fcu`.
2. `views = sensors.Sample(t)`; `world.Update(views, t)`.
3. Collect and handle events (§4). Handling may push behaviours onto the executor (preemption, spec 03 §8.6).
4. `executor.Tick(world, t)`. Its single dispatch goes through the safety filter to the actuator block. Task events go to the journal and the task list.
5. `actuators.Update(t)`: the `fcu` runs its cascade.
6. `resources.Update(world, t)`.

### 3.1a Idle at layer (WP7, JB 2026-10-08)

An airborne robot with nothing to do waits **at its own layer** (`layer.alt`, spec 06 §6), not at the altitude of its last task. After step 4, when the robot is airborne, its executor is empty, no task is pending, new tasks are not blocked, it is not returning home and this has lasted at least 1 s, the controller sends `A_PZY layer.alt yaw` through the safety supervisor, where `yaw` is the heading held when it went idle. The horizontal setpoint is left as it was, so the robot holds its place and only changes altitude. The 1 s delay keeps it from moving between two tasks that follow each other. A robot without `layer.alt` (no mission header) holds where it is.

Effect on the WP6 runs (spec 10; even dispatch, 30 tasks, cluster, multicluster and random families, seeds 1–10, re-run on 2026-10-08): at N = 8, separation breaches per run fell from 16.0, 10.6 and 4.6 to 10.3, 6.3 and 1.5 with exclusive MRS-RTA, and barely changed with open MRS-RTA (41.8, 34.5, 14.6 to 40.1, 32.8, 13.7), whose breaches come from robots sharing a target. Makespans stayed within 1 to 4%. At N = 3 nothing changed beyond noise. These runs needed the travel fix of spec 09 §4 (`Δs3`); without it, exclusive claims flipped without end and 19 of 30 runs at N = 8 stalled.

*Future mode (JB, not in version 0.1):* idle robots land and climb back to their layer only to hear from task dispatch, depending on the radio's range and on whether there is a signal at ground level.

### 3.2 Task list (WP4 only)

Tasks are added from task strings (`.mrst` text). The executor's task source hands out the next one in order unless new tasks are blocked (battery low, swap). A task that fails the static checks (spec 03 §5), or whose target (a `C_P3` anywhere in it) lies outside the geofence shrunk by `fence_margin` or below `min_alt` (§5), ends at once as `FAILED` with reason `IMPOSSIBLE`. The shrunk box is used because the supervisor would clamp the robot short of a target in the margin, and the task's position condition could then never hold. In WP5 the MRS layer replaces this list.

## 4. Events

Events are collected once per tick and handled in this order. Each fires on its edge, not on every tick.

| Event | When | Handling |
|---|---|---|
| `FAULT` | a node is unavailable at the first tick, or the `fcu` enters failsafe | A flight-critical fault (any of `m_fl`…`m_rr`, `fcu`, or `pose.enu` not provided) at the first tick: take no tasks. Failsafe in the air: block new tasks and push `emergency_land`; after landing, stop. Other faults: tasks that do not pass the static checks fail with `IMPOSSIBLE` when they come up |
| `BATTERY_CRITICAL` | `battery.critical` becomes TRUE | Block new tasks; push `emergency_land` unless already landing (no return home). After landing, or at once when on the ground: journal `swap_land` and stop |
| `BATTERY_LOW` | `battery.low` becomes TRUE | Block new tasks. If the energy for the rest of the current task plus the way home plus the reserve is available (§6), let it finish; then, or at once otherwise, push `land` and `return_home` (so the robot flies home, then lands). After landing at home: journal `swap_land` and stop |
| `GEOFENCE` | `geofence.inside` becomes FALSE while airborne | Push `land` and `return_home` |

A preempted task resumes after the pushed behaviours end (spec 03 §8.6), so a task interrupted by `GEOFENCE` lifts off again and continues, and a task interrupted by `BATTERY_LOW` is resumed after the swap (§7). The controller pushes a behaviour as a root task with the id `safety.<name>` (for example `safety.return_home`); these ids are not task ids of the list and are not journaled.

## 5. Safety supervisor

The safety supervisor wraps the actuator block as the executor's sink and filters every dispatched action:

* `A_PXY`: the target is clamped into the geofence box shrunk by `fence_margin` (2 m).
* `A_PZY`: `z` is clamped to `[min_alt, zmax − fence_margin]`, with `min_alt` 0.5 m above home ground while airborne.
* `A_TO`: the altitude is clamped to `zmax − fence_margin`.
* `A_VXY`, `A_VZY`: clamped to `max_speed_xy` and `max_climb`; a horizontal velocity that points out of the fence within `fence_margin` of its edge is zeroed.

A filtered action is still dispatched; the supervisor records how many it changed. It also checks task targets before a task starts (§3.2). Peer separation is WP5 (plan §7.3).

## 6. Resources and energy

`ResourceManager` reads `battery.energy_wh` and the storage capacity, and owns an energy model:

```cpp
class IEnergyModel {
public:
  virtual double Power(const FlightSegment& s) const = 0;   // W
  virtual double Energy(const std::vector<FlightSegment>& plan) const;  // Wh
  virtual void Observe(double power_w, double speed, double climb) {}   // online calibration
};
```

A `FlightSegment` is a duration at a speed and climb rate (hover is speed 0). The default `QuadEnergyModel`:

* `P(v, v_z) = P_hover · (1 + c_v · v²) + k_climb · max(v_z, 0)`, starting from `P_hover` 90 W, `c_v` 0.004 s²/m² and `k_climb = m · g / η` with `m` 0.9 kg and `η` 0.6;
* calibration: over each window of at least 1 s between two battery readings spent airborne, the measured power (the drop in `battery.energy_wh`) and the mean speed and climb update `P_hover`, `P_hover · c_v` and `k_climb` by recursive least squares on the regressors `(1, v², max(v_z, 0))`, with forgetting factor 0.99. A window whose power is below a quarter or above four times the model's value is skipped (a battery swap or a jump in the gauge is not flight power). The climb coefficient is calibrated too, because a platform's climb power can differ a lot from `m · g / η` (the test simulator draws power from motor speed only);
* path cost: straight segments at cruise speed (`max_speed_xy · 0.75`), climbs at `max_climb`, plus hover time.

Feasibility of a task: `cost(here → task) + cost(task) + cost(task → home) + cost(landing) + reserve ≤ remaining`, with `reserve` 15 % of capacity and `remaining` this tick's battery reading. The cost of a task is estimated from its position conditions (`C_P3`, visited in record order) and its `A_W` and `A_HD` durations as hover time; the landing descends at 0.7 m/s at hover power. In the test simulator the estimate for an out-and-back leg with a hold is within 2 % of the battery after a short calibration flight; the WP4 test allows 15 %.

## 7. Journal and resume

The robot controller writes the journal of spec 06 §8 to an output stream:

* `J_H` and `J_E start` at the first tick, once the mission header is known;
* `J_T` for every task state change, with pool state `ACTIVE` while the task is on the stack, `DONE` or `FAILED` when it ends, and `AVAILABLE` for tasks still in the list;
* `J_K` whenever the stack's task ids change (behaviours are not listed);
* `J_E swap_land` when the robot has landed for a battery swap, and `J_E end` when the list is done.

**Resume.** `RobotController::Resume(journal)` reads a journal (a partly written last record is ignored). If its mission equals the current one, the tasks whose last `J_T` is not `DONE` or `FAILED` go back into the task list, in journal order, with `STARTED` and `IN_PROGRESS` reset to `IDLE` and the action iterator kept for the record only; the task that was at the top of the stack goes first. The controller writes `J_E restart`, pushes `liftoff`, and continues. A journal of another mission is ignored and reported.

## 8. UAV behaviour library additions

[`examples/uav_behaviours.mrsb`](examples/uav_behaviours.mrsb) adds:

| Entry | Fulfils | Priority | How |
|---|---|---|---|
| `flyto.altitude_first` | `C_P3` | 4 | `A_PXY`/`A_PZY` from `cruise_x`, `cruise_y`, `cruise_z` with `cruise_alt` 20 m (climb to `max(cruise_alt, target.z)`, cross, descend) |
| `flyto.velocity` | `C_P3` | 3 | `A_VXY`/`A_VZY` from `vel_to_target_*` (gain 0.8, at most 6 m/s across and 1.5 m/s up or down), yaw rate 0 |
| `emergency_land` | `C_? landed` | 1 | `A_HD 1`, then `A_LD 1`; the controller pushes it by name |

New registry functions (spec 02 §8): `cruise_x`, `cruise_y` (coefficients `cruise_alt`, `alt_tol`, `switch_radius`) and `cruise_z` (`cruise_alt`, `switch_radius`), the same shape as `layered_*` with a fixed cruise altitude instead of `layer.alt`.

The robot picks the fly-to variant by priority (the highest wins). `BehaviourLibrary::SetPriority(name, p)` changes one entry's priority in the robot's own copy of the library (for example `flyto.direct` above `flyto.layered` for a lone UAV), and `ByName(name)` finds an entry for the controller to push.

## 9. Test simulator

`tests/flight/QuadSim.h` is a platform (spec 04 §6) with a rigid-body quadrotor for the WP4 tests. It is not part of the library.

* Motors: Webots propeller model, thrust `k_t · |ω| · ω` with `k_t` = 0.00026 and reaction torque `k_q · ω²` with `k_q` = 0.0000052; motor positions from `Mavic2Pro.proto`.
* Body: mass 0.55 kg and inertia `diag(0.010, 0.012, 0.020)` kg·m² (estimates; deliberately not the mass the default `hover_speed` implies, so the hover integrator is exercised), linear and angular drag, flat ground at z = 0.
* Integration at 1 ms inside the 8 ms platform step.
* Ports with the Webots device names: GPS (WGS-84 through the mission's geo reference), inertial unit, gyro, compass (north vector in the body frame), battery (energy from `P = P_0 + k_p · Σ|ω|³`), LEDs.

## 10. Example task set

[`examples/uav_single_flight.mrst`](examples/uav_single_flight.mrst) is the WP4 acceptance flight: take off to 5 m, fly to (40, 20, 10) and flash the LEDs green for 1 s, then land. The fly-to and the landing are start conditions (`C_P3`, `C_? landed`), so the behaviour library supplies them; the tests of spec 07 §5.6 fly it.
