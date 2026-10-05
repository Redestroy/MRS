# 04 Devices and ports

Status: **draft for WP0**, spec version `0.1`. Grammar: [01](01_Protocol_Grammar.md). Plan reference: §4 (port layer) and §5 (device tree).

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
| parameters | `np × (id key, value)` | Parameter pairs. A value is a number, boolean, identifier or string. A key MAY repeat only where the parameter table says so |

Then, depending on the code:

| Code | Further slots |
|---|---|
| `D_H` | `D+` children |
| `D_J` | `D+` children |
| `D_X` | `P*` ports, `K*` capability overrides, `D*` explicit children |
| `D_S`, `D_A`, `D_C`, `D_M` | `P*` ports, `K*` capability overrides |

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

The Webots Mavic as a full tree. The file [`examples/mavic_webots.mrsd`](examples/mavic_webots.mrsd) is the same tree plus a capability override and a port requirement:

```
@: MRS 0.1/
D: D_H uav head.default 3 robot_type mavic2pro role camera_uav id 1 D_1/
D_1: D_J frame mech_bus 0 D_1..4/
D_1: D_X quadrotor quadrotor.webots 0/
D_2: D_A leds led.webots 2 device "front left led" device "front right led"/
D_3: D_M bat battery.webots 1 capacity_wh 50/
D_4: D_C radio radio.webots 2 channel 1 range_m 200/
```

`quadrotor.webots` builds its own virtual branch (§3), so the file does not list motors or sensors.

## 3. Complex nodes and the quadrotor branch

`ComplexDevice::Expand()` creates virtual child nodes from its own parameters. The expanded tree is what the block builder and the self model see.

The `quadrotor.*` complex node expands to:

| Name | Code | Capabilities |
|---|---|---|
| `m_fl`, `m_fr`, `m_rl`, `m_rr` | `D_A` | `A_MOT` |
| `fcu` | `D_A` | `A_TO`, `A_LD`, `A_HD`, `A_PXY`, `A_PZY`, `A_VXY`, `A_VZY`; drives the four motors through `A_MAP` (spec 02 §5) |
| `imu` | `D_S` | views `V_ATT`, `V_RATE`, `V_ACC` |
| `gnss` | `D_S` | view `V_GEO` |
| `baro` | `D_S` | view `V_BARO` |
| `compass` | `D_S` | view `V_MAG` |

Parameters of `quadrotor.webots`, all optional, with Webots device names as defaults:

| Key | Default |
|---|---|
| `motor_fl`, `motor_fr`, `motor_rl`, `motor_rr` | `"front left propeller"`, `"front right propeller"`, `"rear left propeller"`, `"rear right propeller"` |
| `gps`, `imu`, `gyro`, `compass` | `"gps"`, `"inertial unit"`, `"gyro"`, `"compass"` |
| `max_climb`, `max_speed_xy` | 2.0 m/s, 8.0 m/s |

*Note:* the Webots device names above are from memory of the bundled Mavic 2 Pro model and are checked in WP2.

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

* Target `any`: if exactly one actuator accepts the code, it gets the action. If more than one does, the one with parameter `default T` gets it; if none or several are marked, the build fails with an ambiguity error.
* A named target: the named node MUST accept the code, otherwise the action is `REJECTED`.
* `A_N`, `A_W` and `A_I` never reach an actuator (spec 03 §8.2).

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

## 6. Port requirements

| Code | Slots | Meaning |
|---|---|---|
| `P_R` | `id port_type`, `id` or `str` `address`, `bool exclusive`, `int n`, `n × (id key, value)` | A port the node needs. Use a string for names with spaces, such as Webots device names |

* `port_type` ∈ `GPIO PWM ADC UART I2C SPI CAN UDP TCP MAVLINK SIM`.
* `address` is a platform name or address, or `any`.
* Parameter keys depend on the type: `baud` (UART), `bus_addr` (I2C), `msg` (MAVLINK message name; repeatable), `device` (SIM; the Webots device name).

Device classes declare their own port requirements from their parameters. A `P_R` record in a definition file replaces the class's requirement of the same type.

**Assignment** (`PortManager`): for each requirement, in tree order:
1. An exact address or name match.
2. Otherwise the first free port of the type.
3. An exclusive port is never given twice. A conflict is a build error.
4. A node whose required port cannot be assigned is **unavailable**. Its capabilities are left out of the self model and a `FAULT` event is raised (plan §7.3). The robot still starts.

## 7. The self model

The head builds the self model after port assignment. It contains:

* `robot_type`, `id` and `role` list;
* every available `K_A`, `K_V`, `K_M`, `K_Q`, with the node name that provides it;
* the worldview fields the robot can provide: the union of the `Provides()` lists of every processor whose subscribed views appear in the `K_V` list (spec 05 §5).

The self model is sent to peers as a `PROFILE` message (spec 06 §4).
