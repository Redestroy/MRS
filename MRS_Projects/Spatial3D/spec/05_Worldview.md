# 05 Worldview

Status: **draft for WP0, updated in WP2**, spec version `0.1`. Conventions (units, frames, time): [00](00_Conventions.md). Plan reference: §6.

## 1. Model

The worldview holds **fields**. A field is a named, typed, time-stamped value written by a processor (or by the binding step, spec 03 §7) and read by conditions, functions, estimators and allocators.

```cpp
namespace MRS::Environment {
  template <class T> struct WorldField {
    T value;
    double stamp;        // mission time of the information (not of the write)
    SourceId source;     // processor, sensor or peer that produced it
    bool valid;
  };
}
```

Every field also keeps a `TimeSeries` of past values. Its capacity is set per field (§4), with a default of 256 samples.

## 2. Field paths

```ebnf
path     = segment { "." segment } ;
segment  = lower { lower | digit | "_" } ;      (* lower = a..z *)
```

* Segments start with a lower-case letter, so a path never looks like a task id (spec 01 §2.3).
* A **component** of a vector field is addressed by adding its component name: `pose.enu.x`, `geo.position.lat`, `att.yaw`.
* Conditions and `F_L` functions read **scalar** paths only (a scalar field, a component, or a boolean predicate for `C_?`).

## 3. Types, freshness and missing values

| Type | Components | Example |
|---|---|---|
| `scalar` | — | `alt.agl` |
| `bool` | — | `airborne` |
| `vec3` | `x y z` | `pose.enu`, `vel.enu` |
| `geo` | `lat lon alt` | `geo.position` |
| `euler` | `roll pitch yaw` | `att` |
| `battery` | `voltage remaining energy_wh` | `battery` |
| `id` | — | `task.active` |

* A field is **missing** if no processor has written it, or its last write had `valid = false`.
* A field is **stale** if `t − stamp > max_age`. `max_age` defaults per field (§4). A requirement (`R_F`) may set a stricter value.
* Reading a missing or stale field makes a condition UNKNOWN and a function fail (spec 02 §6, spec 03 §4).
* Writers MUST NOT write `nan` or `inf`. A processor that cannot compute a value writes `valid = false`.

## 4. Field registry (version 0.1)

### 4.1 Minimum UAV field set

Every UAV MUST provide these fields to take spatial tasks (JB's worldview minimum: timing, altitude, heading, GNSS position).

| Path | Type | Unit | Default `max_age` | Written by |
|---|---|---|---|---|
| `time` | scalar | s | — (always fresh) | `Clock` |
| `geo.position` | geo | deg, deg, m AMSL | 1.0 s | `GnssProcessor` |
| `pose.enu` | vec3 | m | 0.5 s | `GeoToLocalProcessor` |
| `alt.agl` | scalar | m | 0.5 s | `AltitudeProcessor` |
| `alt.amsl` | scalar | m | 0.5 s | `AltitudeProcessor` |
| `att` | euler | rad | 0.2 s | `AttitudeProcessor` |
| `heading` | scalar | rad, 0 = North, clockwise | 0.2 s | `AttitudeProcessor` |
| `vel.enu` | vec3 | m/s | 0.5 s | `KinematicsEstimator` |
| `acc.enu` | vec3 | m/s² | 0.5 s | `KinematicsEstimator` |
| `rate.body` | vec3 | rad/s | 0.2 s | `AttitudeProcessor` |
| `battery` | battery | V, 0–1, Wh | 2.0 s | `BatteryProcessor` |

*Note:* `att.yaw` uses the ENU yaw convention and `heading` the navigation convention (spec 00 §3). Both exist because tasks written by operators usually think in headings, while control code uses yaw.

### 4.2 Predicates

| Path | TRUE when | Written by |
|---|---|---|
| `airborne` | `alt.agl` > 0.3 m and the motors are running | `FlightStateProcessor` |
| `landed` | `alt.agl` < 0.15 m and `|vel.enu|` < 0.1 m/s for 1 s | `FlightStateProcessor` |
| `armed` | motors armed | `FlightStateProcessor` |
| `home` | within 1 m horizontally of `home.enu`, at any altitude | `FlightStateProcessor` |
| `geofence.inside` | inside the mission geofence | `SafetySupervisor` |
| `battery.low` | `battery.remaining` < `battery_low` (default 0.3) | `BatteryProcessor` |
| `battery.critical` | `battery.remaining` < `battery_critical` (default 0.15) | `BatteryProcessor` |

The thresholds are configuration, not spec.

### 4.3 Mission and target fields

| Path | Type | Written by |
|---|---|---|
| `home.enu` | vec3 | the mission header (spec 06 §6) |
| `layer.alt` | scalar | the mission header: this robot's cruise altitude |
| `target.x`, `target.y`, `target.z` | scalar | the binding step (spec 03 §7) |
| `target.yaw`, `target.tol_xy`, `target.tol_z`, `target.tol_yaw` | scalar | the binding step |
| `target.predicate`, `target.value` | id, bool | the binding step |
| `task.active` | id | the executor |
| `task.elapsed` | scalar s | the executor: time since the top task left `QUEUED` |

`target.*` fields belong to the **innermost bound behaviour**. When it is popped, the previous values are restored.

### 4.4 Optional fields

| Path | Type | Written by | Notes |
|---|---|---|---|
| `peer.<id>.pose.enu`, `peer.<id>.vel.enu` | vec3 | `PeerStateProcessor` | `<id>` is the robot id written as `r<N>`, for example `peer.r3.pose.enu` |
| `peer.<id>.battery.remaining` | scalar | `PeerStateProcessor` | |
| `peer.<id>.task` | id | `PeerStateProcessor` | |
| `rel.<id>.enu` | vec3 | `RelativePositionProcessor` | Relative position of a peer from a ranging sensor |
| `det.<class>.<n>.enu` | vec3 | `DetectionProcessor` | Object detections |

## 5. Processors

```cpp
namespace MRS::Environment {
  class IViewProcessor {
  public:
    virtual std::vector<ViewCode> Subscriptions() const = 0;
    virtual std::vector<std::string> Provides() const = 0;      // field paths
    virtual std::vector<std::string> Needs() const { return {}; }  // fields read
    virtual void Process(const View& v, Worldview& w, double t) = 0;
    virtual void Tick(Worldview& w, double t) {}
  };
}
```

* `ProcessorChain` orders processors once at start so that every processor runs after the processors that provide its `Needs()`. A cycle is a build error.
* A processor writes only the fields in its `Provides()` list. Two processors MUST NOT provide the same field. Pass-through processors for ArduPilot and fusing processors for Webots therefore replace each other; they are never both loaded.
* The self model's field list (spec 04 §7) is the union of `Provides()` over the processors whose subscriptions the robot's sensors can satisfy.

Per tick (plan §6.3): views are routed, `Process` runs per view, then `Tick` runs per processor in chain order, then predicates are recomputed.

### 5.1 Processor catalog (version 0.1)

The catalog describes each processor without instantiating it. The self model (spec 04 §7) resolves it to a fixed point to learn which fields a robot can provide. Entries are in preference order.

| Processor | Subscriptions (any one) | Needs | Provides | Optional outputs |
|---|---|---|---|---|
| `Clock` | none | — | `time` | |
| `GnssProcessor` | `V_GEO` | — | `geo.position` | |
| `GeoToLocalProcessor` | none | `geo.position` | `pose.enu` | |
| `LocalPositionProcessor` | `V_POS3` | — | `pose.enu` | |
| `AltitudeProcessor` | `V_BARO`, `V_GEO`, `V_RNG` | — | `alt.amsl`, `alt.agl` | |
| `AttitudeProcessor` | `V_ATT` | — | `att`, `heading` | `rate.body` with `V_RATE` |
| `KinematicsEstimator` | none | `pose.enu` | `vel.enu`, `acc.enu` | |
| `BatteryProcessor` | `V_BAT` | — | `battery`, `battery.low`, `battery.critical` | |
| `FlightStateProcessor` | none | `alt.agl`, `vel.enu` | `airborne`, `landed`, `armed`, `home` | |
| `SafetySupervisor` | none | `pose.enu` | `geofence.inside` | |
| `PeerStateProcessor` | `V_PEER` | — | `peer` (the `peer.<id>.*` subtree) | |

* `AltitudeProcessor` prefers `V_RNG` for `alt.agl` and `V_BARO` for `alt.amsl`, and falls back to the `V_GEO` altitude, so a robot without a barometer (the stock Webots Mavic) still has altitude.
* `AttitudeProcessor` takes `heading` from `V_MAG` when present and from the `V_ATT` yaw otherwise.
* `GeoToLocalProcessor` comes before `LocalPositionProcessor`, so a robot with both GNSS and a local position source takes `pose.enu` from GNSS. Pass-through catalogs for ArduPilot (WP9) list their own processors in their own order.

## 6. Views

A view is one piece of raw information from a sensor, or a decoded peer message. Its text form is used in messages, logs and `C_V` conditions.

Every view code starts with the slot `num stamp` (mission time of the information).

| Code | Slots after `stamp` | Produced by |
|---|---|---|
| `V_GEO` | `num lat`, `num lon`, `num alt_amsl` | GNSS |
| `V_POS3` | `num x`, `num y`, `num z` | ENU position source (for example ArduPilot local position) |
| `V_VEL3` | `num vx`, `num vy`, `num vz` | velocity source |
| `V_ATT` | `num roll`, `num pitch`, `num yaw` | IMU / inertial unit |
| `V_RATE` | `num p`, `num q`, `num r` | gyro |
| `V_ACC` | `num ax`, `num ay`, `num az` | accelerometer (body frame) |
| `V_BARO` | `num alt_amsl` | barometer |
| `V_MAG` | `num heading` | compass |
| `V_RNG` | `num range` | downward rangefinder |
| `V_BAT` | `num voltage`, `num remaining`, `num energy_wh` | battery |
| `V_PEER` | `int robot`, `num x`, `num y`, `num z`, `num vx`, `num vy`, `num vz`, `num battery`, `tid task` | a peer's `STATE` message (spec 06) |
| `V_REL3` | `int robot`, `num dx`, `num dy`, `num dz` | relative-position sensor |
| `V_DET` | `id class`, `num x`, `num y`, `num z`, `num confidence` | detector |
| `V_P2` | `num x`, `num y`, `num yaw` | legacy 2D pose (E-puck) |

**View matching (`C_V`)** in version 0.1 is defined only for:
* `V_DET`: TRUE when the worldview holds a detection of the same class within 2 m of the given position. The stamp in the condition is ignored.
* `V_PEER`: TRUE when the worldview holds fresh state for that robot id. The other slots are ignored.

Other view codes in a `C_V` condition are a parse error in version 0.1; use `C_P3`, `C_H` or `C_m` instead.
