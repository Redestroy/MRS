# 04 Devices and ports

Status: **draft for WP0, updated in WP2**, spec version `0.1`. Grammar: [01](01_Protocol_Grammar.md). Plan reference: §4 (port layer) and §5 (device tree).

## 1. Device tree

A robot's devices form a tree that follows their physical connections.

| Node kind | Code | May have children | Role |
|---|---|---|---|
| Head | `D_H` | yes (D) | Root. Runs the robot controller, builds the self model. Exactly one per tree in version 0.1 |
| Joint | `D_J` | yes (D) | A link with an interface and connection rules |
| Complex | `D_X` | yes (D), plus the virtual branch it builds itself | One physical block represented as a branch of nodes |
| Sensor | `D_S` | no | Edge. Capability: views |
| Actuator | `D_A` | no | Edge. Capability: actions |
| Communication | `D_C` | no | Edge. Capability: messages |
| Storage | `D_M` | no | Edge. Capability: capacity (energy, memory, payload) |

Rules:
* The root MUST be `D_H`.
* Edge nodes MUST NOT have children.
* Node names are identifiers, unique within the whole tree (they are action targets, spec 02 §5).
* A joint's connection rules (§2.2) MUST hold for its children. Violations are errors at build time.

## 2. Definition file (`.mrsd`)

### 2.1 Common slots

Every `D` code has the same leading slots:

| Slot | Type | Meaning |
|---|---|---|
| `name` | `id` | Node name |
| `type` | `id` | Registry key of the class that implements it (§5), for example `gnss.webots`. For `D_J`, the interface kind (§2.2) |
| `np` | `int` | Number of parameters |
| parameters | `np × (id key, value)` | Parameter pairs. A value is a number, boolean, identifier, code (for example `accepts D_S`) or string. A key MAY repeat only where the parameter table says so |

Then, depending on the code:

| Code | Further slots |
|---|---|
| `D_H` | `D+` children |
| `D_J` | `D+` children |
| `D_X` | `P*` port requirement overrides, `K*` capability overrides, `D*` explicit children |
| `D_S`, `D_A`, `D_C`, `D_M` | `P*` port requirement overrides, `K*` capability overrides |

### 2.2 Joints

| Interface kind (`type`) | Carries |
|---|---|
| `mech` | Mechanical mount only |
| `bus` | Data bus only (for example I²C, CAN) |
| `elec` | Power only |
| `mech_bus` | Mount and data bus |
| `mech_bus_elec` | All three |

Joint parameters:

| Key | Value | Repeatable | Default |
|---|---|---|---|
| `max_children` | int, 0 = unlimited | no | 0 |
| `accepts` | a node code (`D_S`, `D_A`, `D_C`, `D_M`, `D_X`, `D_J`) | yes | all |
| `frame` | string `"x y z roll pitch yaw"`, the mount transform in the parent's body frame | no | identity |

### 2.3 Head parameters

| Key | Value | Repeatable | Meaning |
|---|---|---|---|
| `robot_type` | id | no | Robot type, used by role requirements |
| `role` | id | yes | Roles the robot can take (JB decision 9) |
| `id` | int | no | Robot id in the team (≥ 1). Also sets the default altitude layer |

### 2.4 Example

The Webots Mavic as a full tree. The file [`examples/mavic_webots.mrsd`](examples/mavic_webots.mrsd) is the same tree plus a capability override and a port requirement override:

```
@: MRS 0.1/
D: D_H uav head.default 3 robot_type mavic2pro role camera_uav id 1 D_1/
D_1: D_J frame mech_bus 0 D_1..6/
D_1: D_X quadrotor quadrotor.webots 0/
D_2: D_S gnss gnss.webots 0/
D_3: D_S compass compass.webots 0/
D_4: D_A leds led.webots 2 device "front left led" device "front right led"/
D_5: D_M bat battery.webots 1 capacity_wh 50/
D_6: D_C radio radio.webots 2 channel 1 range_m 200/
```

`quadrotor.webots` builds its own virtual branch of motors, flight control unit and IMU (§3). Position and heading sensors are separate lines, so that removing a sensor from the file removes what it provides from the self model (§7).

The stock Webots Mavic 2 Pro has no emitter and no receiver. The radio needs an `Emitter` and a `Receiver` added to the robot's `bodySlot` field in the world file, named `"emitter"` and `"receiver"`.

## 3. Complex nodes and the quadrotor branch

`ComplexDevice::Expand()` creates virtual child nodes from its own parameters. The expanded tree is what the block builder and the self model see. Explicit `D` children of a `D_X` follow the virtual ones.

The `quadrotor.*` complex node expands to:

| Name | Code | Registry key (Webots) | Capabilities |
|---|---|---|---|
| `m_fl`, `m_fr`, `m_rl`, `m_rr` | `D_A` | `motor.webots` | `A_MOT` |
| `fcu` | `D_A` | `fcu.default` | `A_TO`, `A_LD`, `A_HD`, `A_PXY`, `A_PZY`, `A_VXY`, `A_VZY`, with limits `max_climb` and `max_speed_xy`; drives the four motors through `A_MAP` (spec 02 §5). It is unavailable when any motor is unavailable |
| `imu` | `D_S` | `imu.webots` | views `V_ATT`, `V_RATE`, and `V_ACC` only when an accelerometer is named |

Virtual node names are fixed, so a tree has at most one quadrotor in version 0.1.

Parameters of `quadrotor.webots`, all optional, with the Webots Mavic 2 Pro device names as defaults (checked against the Cyberbotics `Mavic2Pro.proto` in WP2):

| Key | Default |
|---|---|
| `motor_fl`, `motor_fr`, `motor_rl`, `motor_rr` | `"front left propeller"`, `"front right propeller"`, `"rear left propeller"`, `"rear right propeller"` |
| `inertial`, `gyro` | `"inertial unit"`, `"gyro"` |
| `accelerometer` | none. The stock Mavic has no accelerometer; one added to `bodySlot` is named here |
| `max_climb`, `max_speed_xy` | 2.0 m/s, 8.0 m/s |
| `rate_hz` | sensor sampling rate; default: the platform's basic step |

### 3.1 UAV edge devices

| Registry key (Webots) | Code | Parameters (defaults) | Capabilities | Port requirements (§6) |
|---|---|---|---|---|
| `head.default` | `D_H` | §2.3 | none | none |
| `gnss.webots` | `D_S` | `device` (`"gps"`), `frame` (`wgs84` or `local`, default `wgs84`), `rate_hz` | `V_GEO`; `V_POS3` with `frame local` | `gnss` |
| `compass.webots` | `D_S` | `device` (`"compass"`), `rate_hz` | `V_MAG` | `compass` |
| `baro.webots` | `D_S` | `device` (`"altimeter"`), `rate_hz` | `V_BARO` | `baro` |
| `led.webots` | `D_A` | `device`, repeatable, one per LED bit (`"front left led"`, `"front right led"`); `rgb` (`F`: any colour turns the LED on; `T`: the colour is written to an RGB LED) | `A_L` | `led0`, `led1`, … |
| `battery.webots` | `D_M` | `capacity_wh` (required), `voltage` (nominal, 11.55 V) | `K_Q energy_wh`, view `V_BAT` | `battery` (address `battery`, the `Robot.battery` field) |
| `radio.webots` | `D_C` | `channel` (1), `range_m`, `emitter` (`"emitter"`), `receiver` (`"receiver"`) | `K_M broadcast` | `tx`, `rx` |

The stock Mavic has no barometer; without `baro.webots` the worldview takes `alt.amsl` from GNSS (spec 05 §5.1).

Compass heading: Webots returns the north direction in the body frame, `(n_x, n_y, n_z)`. The heading is `atan2(n_y, n_x)` wrapped to `[0, 2π)`.

A storage device MAY produce views (the battery produces `V_BAT`); it is then sampled like a sensor.

## 4. Capabilities

### 4.1 Records

| Code | Slots | Meaning |
|---|---|---|
| `K_A` | `code action`, `int n`, `n × (id key, num value)` limits | The node accepts this action code |
| `K_V` | `code view`, `int n`, limits | The node produces this view code |
| `K_M` | `id mode` (`broadcast` or `p2p`), `int n`, limits | The node sends and receives messages |
| `K_Q` | `id resource`, `num capacity`, `int n`, limits | The node stores a resource (`energy_wh`, `memory_kb`, `payload_kg`) |

Common limit keys: `rate_hz`, `max`, `min`, `range_m`, `bitrate_bps`, `max_speed_xy`, `max_climb`, `max_alt`.

Capabilities are normally declared by the device class. A `K` record in a definition file **overrides the limits** of a capability the class declares; it cannot add a capability the class does not have.

### 4.2 Action dispatch

The block builder indexes actuators by **action code**.

* Target `any`: if exactly one available actuator accepts the code, it gets the action. If more than one does, the one with parameter `default T` gets it. If none of them is marked, the code has no `any` route and an `any` action with it is `REJECTED` (the four motors share `A_MOT` and are always addressed by name). Two actuators marked `default T` for the same code is a build error.
* A named target: the named node MUST exist, be available and accept the code, otherwise the action is `REJECTED`.
* A combined action is checked as a whole before any entry is applied: if one entry has no route, nothing is applied and the map is `REJECTED`. Otherwise every entry is applied in the same tick and the map's status is `FAILED` if any entry failed, else `RUNNING` if any entry is running, else `DONE`.
* `A_N`, `A_W` and `A_I` never reach an actuator (spec 03 §8.2). If one does, it is `REJECTED`.

## 5. Device registry

```cpp
namespace MRS::Device {
  class DeviceRegistry {
  public:
    template <class T> void Register(std::string type_key);   // T derives from DeviceNode
    std::unique_ptr<DeviceNode> Create(const std::string& type_key, const Params&) const;
  };
}
```

* Type keys are `<device>.<platform>`, for example `gnss.webots`, `gnss.mavlink`, `quadrotor.webots`, or `<device>.default` when no platform code is involved (`head.default`).
* An unknown type key is a build error.

## 6. Ports

Ports of the same type are not interchangeable: which device sits on which UART, I²C address or Webots device name is fixed by the wiring or by the connection order. A port is therefore never picked because it is free. It comes from the robot's **port map** (first-time setup, §6.2), from an explicit address, or from a **port scan** that identifies the device (§6.3).

### 6.1 Port requirements

Device classes declare their port requirements from their parameters. Each requirement has a **name** that is local to the node (`gnss`, `tx`, `led1`; the tables in §3).

| Code | Slots | Meaning |
|---|---|---|
| `P_R` | `id port_type`, `id` or `str` `address`, `bool exclusive`, `int n`, `n × (id key, value)` | Overrides one of the node's port requirements: the address and exclusive flag are replaced, and each parameter replaces the class's parameter of the same key. Use a string for names with spaces, such as Webots device names |

* `port_type` ∈ `GPIO PWM ADC UART I2C SPI CAN UDP TCP MAVLINK SIM`.
* `address` is a platform name or address, or `any` (§6.3).
* Parameter keys depend on the type: `baud` (UART), `bus_addr` (I2C), `msg` (MAVLINK message name; repeatable), `rate_hz` (any; the sampling or update rate), `identity` (any; what a scan must report, §6.3).
* `req` (id): the name of the requirement to override. It MAY be left out when the node has exactly one requirement of that type. A `P_R` that matches no requirement, or more than one, is a build error.

### 6.2 Port map (`.mrsp`, first-time setup)

The port map is a separate file, one `P_A` record per assigned requirement. It is written once per robot, by hand or from a port scan (§6.4), and is the normal source of ports on real hardware.

| Code | Slots | Meaning |
|---|---|---|
| `P_A` | `id node`, `id requirement`, `id port_type`, `id` or `str` `address`, `int n`, `n × (id key, value)` | Requirement `requirement` of node `node` uses this port. The parameters (rates, baud) replace the requirement's parameters of the same key |

```
@: MRS 0.1/
P: P_A gnss gnss SIM "gps" 1 rate_hz 31.25/

P: P_A radio tx SIM "emitter" 0/
```

* `address` MUST NOT be `any`.
* A `P_A` whose node or requirement does not exist, or whose `port_type` differs from the requirement's, is a build error.

### 6.3 Assignment

`PortManager` assigns each requirement, in tree order (depth first), from the first source that applies:

1. the port map entry for (node, requirement);
2. the requirement's own address, when it is not `any`;
3. a port scan: `any` with an `identity` parameter is resolved to the one scanned port of that type whose reported identity equals it.

Then:

* If the platform's scan lists ports of the type, the address MUST be one of them.
* A requirement with `any` and no `identity`, or an identity that matches no port or more than one, is **unresolved**.
* An exclusive port is never given twice. Two requirements on one port where either is exclusive is a build error.
* The platform then opens each assigned port with its parameters. A requirement that is unresolved or whose port does not open makes the node **unavailable**, as does an unavailable node it depends on (the `fcu` depends on the motors). Its capabilities are left out of the self model and a `FAULT` event is raised (plan §7.3). The robot still starts.

### 6.4 First-time setup

`PortManager::WritePortMap()` writes the port map from an assignment: one `P_A` per assigned requirement, with the address it got and the parameters it was opened with. On real hardware the usual setup is: describe every unknown port as `any` with an `identity`, run a scan, check the result and save it as the robot's `.mrsp`. From then on the map is the source and no scan is needed.

## 7. The self model

The head builds the self model after port assignment. It contains:

* `robot_type`, `id` and `role` list;
* every capability of every **available** node (`K_A`, `K_V`, `K_M`, `K_Q`), with the node name that provides it;
* the views the robot produces: every `K_V`, plus `V_PEER` when it has any `K_M` (peer `STATE` messages decode to `V_PEER`);
* the worldview fields the robot can provide, from the processor catalog (spec 05 §5.2).

The field list is computed to a fixed point: start with `time`; a catalog processor is **active** when at least one of its subscribed views is produced (or it subscribes to none) and every field in its `Needs()` is already provided; an active processor adds its `Provides()`, its `Offers()` fields, and those of its optional outputs whose view is produced. Repeat until nothing changes. An offered field is provided when any of its sources is active; the worldview picks among them at run time (spec 05 §5.1).

`ToProfile()` gives the task layer's capability profile: action codes, fields and roles.

The self model is sent to peers as a `PROFILE` message (spec 06 §4).

## 8. Notes for later versions

* **Richer device metadata (JB, 2026-10-06).** Later versions may let devices report more detailed metadata, such as model, interface, rates and calibration, which would allow better self-configuration of the tree and the port map. This is beyond the current work packages. In version 0.1 the definition file, the port map and the `identity` a scan reports are the only sources.
