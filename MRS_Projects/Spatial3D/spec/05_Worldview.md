# 05 Worldview

Status: **draft for WP0, updated in WP2 and WP3**, spec version `0.1`. Conventions (units, frames, time): [00](00_Conventions.md). Plan reference: §6.

## 1. Model

The worldview holds **fields**. A field is a named, typed, time-stamped value written by a processor (or by the binding step, spec 03 §7) and read by conditions, functions, estimators and allocators.

```cpp
namespace MRS::Environment {
  template <class T> struct WorldField {
    T value;
    double stamp;        // mission time of the information (not of the write)
    SourceId source;     // processor, sensor, peer, or the selected source of an offered field (§5.1)
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
| `airborne` | `alt.agl` > 0.3 m and the motors are running (`armed`, when known) | `FlightStateProcessor` |
| `landed` | `alt.agl` < 0.15 m and `|vel.enu|` < 0.1 m/s for 1 s | `FlightStateProcessor` |
| `armed` | motors armed | the flight control unit (WP4) |
| `home` | within 1 m horizontally of `home.enu`, at any altitude | `FlightStateProcessor` |
| `geofence.inside` | inside the mission geofence | `GeofenceProcessor` |
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
| `payload.id` | scalar | `PayloadProcessor` | The package the robot carries, 0 when its bay is empty (WP7, spec 11 §3.1) |

## 5. Processors

```cpp
namespace MRS::Environment {
  struct Offer { std::string field; std::string source; };   // a field this processor offers as one source

  class IViewProcessor {
  public:
    virtual std::string Name() const = 0;
    virtual std::vector<std::string> Subscriptions() const = 0;   // view codes
    virtual std::vector<std::string> Provides() const { return {}; }  // fields only this processor writes
    virtual std::vector<Offer> Offers() const { return {}; }          // fields offered as a source (§5.1)
    virtual std::vector<std::string> Needs() const { return {}; }     // fields read
    virtual std::vector<std::pair<std::string, std::string>> Optional() const { return {}; }  // (view, field)
    virtual void Process(const View& v, Worldview& w, double t) {}
    virtual void Tick(Worldview& w, double t) {}
  };
}
```

* Processors are **simple**: each turns one kind of input into one kind of output. Where a field can come from more than one input (altitude, heading, position), each input has its own small processor that **offers** the field as a named source, and the worldview decides which source to use (§5.1, JB 2026-10-06).
* `ProcessorChain` orders processors once at start, so that every processor runs after the processors that provide or offer its `Needs()`. A cycle is a build error.
* A field in `Provides()` is written by that processor only. Two processors MUST NOT provide the same field, and a field MUST NOT be both provided and offered. Either is a build error.
* Adding or removing a processor needs no change to any other processor: the chain is re-ordered and the source selection uses whatever sources remain.

Per tick (plan §6.3): views are routed to their subscribers and `Process` runs per view; then `Tick` runs per processor in chain order; then the worldview re-selects every offered field (§5.1), so a source that went stale during the tick is replaced. Predicates are written by their processors' `Tick`, which runs after the fields they read.

### 5.1 Source selection

For every offered field the worldview keeps the latest value of each source and writes the field itself from the **best fresh source**:

* A source is fresh when its latest value is at most the field's `max_age` old.
* Sources are ranked by the field's **source order**, a per-robot setting. Sources missing from the order rank after it, by name.
* The field is re-selected whenever a source offers a new value, and at the end of every tick. When no source is fresh, the newest value is kept, so the field reads as stale.
* The field records which source it came from (`WorldField::source`), and the change of source is visible to logs.

Default source orders (version 0.1, UAV):

| Field | Sources, best first | Offered by |
|---|---|---|
| `pose.enu` | `gnss`, `local` | `GeoToLocalProcessor` (from `geo.position`), `LocalPositionProcessor` (`V_POS3`) |
| `alt.amsl` | `baro`, `gnss`, `local` | `BaroAltitudeProcessor` (`V_BARO`), `GnssAltitudeProcessor` (`geo.position.alt`), `LocalAltitudeProcessor` (`V_POS3` z + `alt0`) |
| `alt.agl` | `range`, `amsl` | `RangeAltitudeProcessor` (`V_RNG`), `AglFromAmslProcessor` (`alt.amsl` − ground) |
| `heading` | `compass`, `attitude` | `CompassHeadingProcessor` (`V_MAG`), `AttitudeHeadingProcessor` (`V_ATT` yaw, converted, spec 00 §3) |
| `vel.enu` | `nav`, `kinematics` | `VelocityProcessor` (`V_VEL3`, an autopilot's estimate, spec 14 §5), `KinematicsEstimator` (from `pose.enu`) |

* Ground altitude in version 0.1 is flat ground at the robot's home: `alt0 + home.enu.z`, or `alt0` before the mission header arrives.
* A robot without a barometer (the stock Webots Mavic) gets `alt.amsl` from GNSS; one with a rangefinder gets `alt.agl` from it. Nothing else changes.

### 5.2 Processor catalog (version 0.1)

The self model (spec 04 §7) works out which fields a robot can provide from descriptions of the processors (`Name`, `Subscriptions`, `Needs`, `Provides`, `Offers`, `Optional`), taken from the processor classes themselves, without running them. Entries are in chain-registration order.

| Processor | Subscriptions | Needs | Provides | Offers (field/source) |
|---|---|---|---|---|
| `Clock` | — | — | `time` | |
| `GnssProcessor` | `V_GEO` | — | `geo.position` | |
| `GeoToLocalProcessor` | — | `geo.position` | | `pose.enu`/`gnss` |
| `LocalPositionProcessor` | `V_POS3` | — | | `pose.enu`/`local` |
| `BaroAltitudeProcessor` | `V_BARO` | — | | `alt.amsl`/`baro` |
| `GnssAltitudeProcessor` | — | `geo.position` | | `alt.amsl`/`gnss` |
| `LocalAltitudeProcessor` | `V_POS3` | — | | `alt.amsl`/`local` |
| `RangeAltitudeProcessor` | `V_RNG` | — | | `alt.agl`/`range` |
| `AglFromAmslProcessor` | — | `alt.amsl` | | `alt.agl`/`amsl` |
| `AttitudeProcessor` | `V_ATT` | — | `att` | |
| `RateProcessor` | `V_RATE` | — | `rate.body` | |
| `CompassHeadingProcessor` | `V_MAG` | — | | `heading`/`compass` |
| `AttitudeHeadingProcessor` | `V_ATT` | — | | `heading`/`attitude` |
| `KinematicsEstimator` | — | `pose.enu` | `acc.enu` | `vel.enu`/`kinematics` |
| `VelocityProcessor` | `V_VEL3` | — | | `vel.enu`/`nav` (WP10, spec 14 §5) |
| `BatteryProcessor` | `V_BAT` | — | `battery`, `battery.low`, `battery.critical` | |
| `FlightStateProcessor` | — | `alt.agl`, `vel.enu` | `airborne`, `landed`, `home` | |
| `GeofenceProcessor` | — | `pose.enu` | `geofence.inside` | |
| `PeerStateProcessor` | `V_PEER` | — | `peer` (the `peer.<id>.*` subtree) | |
| `DetectionProcessor` | `V_DET` | — | `det` (the `det.<class>.<n>.*` subtree) | |
| `PayloadProcessor` | `V_PAY` | — | `payload.id` | |

A processor is in the robot's chain when at least one of its subscriptions is produced (or it has none) and its needs are met; this is the same fixed point as spec 04 §7. Processors whose inputs never appear are left out, so they cost nothing.

### 5.3 Processor rules (version 0.1)

* `KinematicsEstimator`: an alpha-beta filter per axis over `pose.enu` (α = 0.85, β = 0.3 by default), run once per new `pose.enu` stamp. `vel.enu` is valid from the second sample; `acc.enu` is the low-pass filtered difference of `vel.enu` (γ = 0.3), valid from the third.
* `FlightStateProcessor`: `airborne` and `landed` as in §4.2, with `armed` treated as unknown. `armed` itself comes from the flight control unit's state in WP4. `home` reads `home.enu` and is invalid while `home.enu` is missing.
* `GeofenceProcessor`: the box of the mission header (spec 06 §6), stored as the mission fields `geofence.xmin`, `.xmax`, `.ymin`, `.ymax` and `.zmax`. `geofence.inside` is invalid while they are missing.
* `PeerStateProcessor`: a `V_PEER` view from robot `N` writes `peer.rN.pose.enu`, `peer.rN.vel.enu`, `peer.rN.battery.remaining` and `peer.rN.task`, stamped with the view's stamp, and the peer's semantic object.
* `DetectionProcessor`: a `V_DET` of class `c` updates the detection of class `c` within 2 m of it, or starts the next number `n`; it writes `det.c.n.enu` and `det.c.n.confidence`.
* Mission fields (`home.enu`, `layer.alt`, `geofence.*`) and the geo reference are written from the mission header by `ApplyMissionHeader` (spec 06 §6).

### 5.4 Time series and semantic objects

* Every scalar field and component keeps a `TimeSeries` (§1) with `Latest()`, `At(t)` (linear interpolation between samples, nothing outside them) and `Window(t0, t1)`. A sample older than the newest is ignored; one with the same stamp replaces it.
* `SemanticObject` is something in the world: `id`, `class` (`self`, `peer`, `target`, `obstacle`, `detection`), ENU position and velocity, last-seen stamp, source and confidence. Peers and detections are kept both as fields (for conditions) and as objects (for allocators and logs).

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
| `V_PAY` | `num package` | payload bay: the package held, 0 for none (WP7) |
| `V_P2` | `num x`, `num y`, `num yaw` | legacy 2D pose (E-puck) |

**View matching (`C_V`)** in version 0.1 is defined only for:
* `V_DET`: TRUE when the worldview holds a detection of the same class within 2 m of the given position. The stamp in the condition is ignored.
* `V_PEER`: TRUE when the worldview holds fresh state for that robot id. The other slots are ignored.

Other view codes in a `C_V` condition are a parse error in version 0.1; use `C_P3`, `C_H` or `C_m` instead.
