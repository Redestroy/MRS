# 02 Actions and functions

Status: **draft for WP0**, spec version `0.1`. Grammar: [01](01_Protocol_Grammar.md). Decisions applied: JB 2026-10-04 decision 10 (one 64-bit argument, combined actuators take a map of actions) and decision 5 (parametric actions bound to worldview fields).

## 1. The action model

An **action** is the smallest command a task gives to the device layer. It has:

* an **action code** (`A_PXY`), which also decides which actuator capability can execute it (spec 04 §4), and
* **one 64-bit argument**.

```cpp
namespace MRS::Device {
  struct Action {
    ActionCode code;     // enum, one value per registry row (§4)
    std::uint64_t arg;   // interpreted through the code's layout (§2)
  };
}
```

Actions stay simple. When an actuator needs a new kind of command, a new action code is added to the registry; an existing code is never overloaded with a second meaning.

## 2. Argument layouts

Each action code has exactly one **layout**. The layout decides how many values the text form has and how they are packed into the 64 bits.

| Layout | Text values | Packing into `arg` |
|---|---|---|
| `NONE` | 0 | `arg = 0` |
| `I64` | 1 integer | two's-complement int64 |
| `F64` | 1 number | IEEE-754 binary64 bits |
| `F32X2` | 2 numbers | high 32 bits = binary32 of value 1; low 32 bits = binary32 of value 2 |
| `I32X2` | 2 integers | high 32 bits = int32 value 1; low 32 bits = int32 value 2 (two's complement) |
| `U32X2` | 2 integers | high 32 bits = uint32 value 1; low 32 bits = uint32 value 2 |

Rules:
* **Value 1 is always the high half.** This matches the existing `Action::FirstHalfAsDouble` / `SecondHalfAsDouble` helpers, which keep their names.
* Converting a number to binary32 rounds to nearest, ties to even. A value outside the binary32 range is an error, not an infinity.
* An integer outside its 32-bit range is an error.
* The text form MUST have exactly the number of values the layout states.

*Note:* binary32 has 24 bits of mantissa, so ENU positions keep about 1 mm resolution up to 8 km from the origin, which is enough for the local frame. Geodetic coordinates never travel in actions; they are converted to ENU first (spec 05 §5).

## 3. Text form

```
A_<code> <value>{0,2}
```

Examples:

```
A_1: A_N/                     # null action
A_2: A_W 2.5/                 # wait 2.5 s
A_3: A_PXY 120 -40/           # horizontal position setpoint (120, -40) m ENU
A_4: A_L 3 16711680/          # LED mask 0b11, colour 0xFF0000
```

## 4. Action code registry (version 0.1)

### 4.1 Generic actions (every robot)

| Code | Name | Layout | Values | Meaning |
|---|---|---|---|---|
| `A_N` | NULL | `NONE` | | Ends the task (spec 03 §8). Never sent to an actuator |
| `A_W` | WAIT | `F64` | duration s | Completes when at least `duration` seconds have passed since the action started. Handled by the executor, not an actuator |
| `A_I` | IMPOSSIBLE | `NONE` | | The task cannot be done by this robot; the executor ends it as `FAILED` with reason `IMPOSSIBLE` |
| `A_L` | LED | `U32X2` | mask, colour `0xRRGGBB` | Set the LEDs selected by the bit mask. Colour 0 is off |

### 4.2 Flight actions (the generic flight-capable machine)

Layers 2 and 3 use only these for flight (plan §5.7). Positions and velocities are ENU, angles are ENU yaw.

| Code | Name | Layout | Values | Meaning |
|---|---|---|---|---|
| `A_TO` | TAKEOFF | `F32X2` | altitude AGL m, climb rate m/s | Arm if needed and climb to the altitude |
| `A_LD` | LAND | `F32X2` | descent rate m/s, 0 | Land at the current position and disarm |
| `A_HD` | HOLD | `F32X2` | duration s, 0 | Hold the current position and yaw. With duration > 0 the actuator reports `RUNNING` until it has passed; with 0 it reports `DONE` at once and the hold lasts until the next setpoint |
| `A_PXY` | POS_XY | `F32X2` | x m, y m | Horizontal position setpoint |
| `A_PZY` | POS_Z_YAW | `F32X2` | z m, yaw rad | Vertical position and yaw setpoint |
| `A_VXY` | VEL_XY | `F32X2` | vx m/s, vy m/s | Horizontal velocity setpoint |
| `A_VZY` | VEL_Z_YAWRATE | `F32X2` | vz m/s, yaw rate rad/s | Vertical velocity and yaw-rate setpoint |

A position setpoint and a velocity setpoint MUST NOT be mixed in one combined action. Missing halves of a setpoint keep their last commanded value: `A_PXY` alone keeps the last `z` and yaw.

### 4.3 Device-level actions (inside the device layer only)

These are produced by combined actuators for their sub-actuators. Tasks SHOULD NOT use them.

| Code | Name | Layout | Values | Meaning |
|---|---|---|---|---|
| `A_MOT` | MOTOR_SPEED | `F64` | angular speed rad/s (signed) | Rotor or wheel speed |

### 4.4 Structural actions

| Code | Name | Slots | Meaning |
|---|---|---|---|
| `A_MAP` | Combined action | `(id A)+` | A map from **target** to action. All entries are dispatched in the same tick (§5) |
| `A_FN` | Parametric action | `code F+` | An action of the given code whose values are computed by functions each tick (§6) |

### 4.5 Legacy codes

The codes below exist in the 2021 library and E-puck app. In version 0.1 they are **reserved**: the parser rejects them, and the converter (spec 00 §8) maps them.

`A_l` virtual digital, `A_p` PWM, `A_t` text output, `A_f` forward, `A_b` backward, `A_r` clockwise, `A_y` counter-clockwise, `A_c` arc drive, `A_d` raw drive, `A_D` dash, `A_M` move, `A_R` turn, `A_K` kick, `A_T` tackle, `A_C` catch, `A_V` turn viewport.

## 5. Combined actions (`A_MAP`)

```
A_1: A_MAP any A_1 any A_2/
A_1: A_PXY 120 -40/
A_2: A_PZY 15 1.57/
```

* Slots are pairs of a **target** and an action label.
* The target is either the name of a device node in the robot's tree (spec 04), for example `m_fl`, or the identifier `any`, which routes the entry by its action code (spec 04 §4).
* Entries MUST be leaf actions or parametric actions (`A_FN`). Nested `A_MAP`, `A_N`, `A_W` and `A_I` are not allowed inside a map.
* Two entries MUST NOT address the same target with the same code.
* The executor delivers the whole map in one tick. If any entry has no actuator that accepts it, the whole map is rejected and the task fails with reason `NO_ACTUATOR`.

Example from the Webots flight controller (device layer, not in tasks):

```
A_1: A_MAP m_fl A_1 m_fr A_2 m_rl A_3 m_rr A_4/
A_1: A_MOT 412.5/
A_2: A_MOT -409.0/
A_3: A_MOT -415.2/
A_4: A_MOT 411.8/
```

## 6. Parametric actions (`A_FN`)

```
A_1: A_FN A_PXY F_1 F_2/
F_1: F_L 1 1 0 target.x/
F_2: F_L 1 1 0 target.y/
```

This reads `target.x` and `target.y` from the worldview each tick (`1·w[x] + 0`).

* The first slot is the action code to produce. It MUST NOT be `A_MAP`, `A_FN`, `A_N` or `A_I`.
* Then one function per text value of that code's layout: 1 for `I64`/`F64`, 2 for the `X2` layouts, 0 for `NONE`.
* **Evaluation:** the executor evaluates every function of the active action **every tick**, immediately before dispatch, with the current worldview and mission time (spec 03 §8). The results are packed by the layout rules (§2).
* Integer layouts round half away from zero, then saturate to the integer range.
* If a function cannot be evaluated (a field it reads is missing or stale, spec 05 §3), the action is not dispatched and the task fails with reason `MISSING_FIELD`.

## 7. Functions

A function computes one `double` from the worldview `w` and the mission time `t`. Functions are the "coefficients tied to worldview fields" of JB's design.

| Code | Name | Slots | Value |
|---|---|---|---|
| `F_K` | Constant | `num` | the number |
| `F_L` | Linear | `int n`, `num×n` coefficients `k`, `num c`, `id×n` fields `x` | `c + Σ kᵢ·w[xᵢ]` |
| `F_X` | Registry | `id key`, `int n`, `num×n` coefficients | `R[key](w, t, k)`, from the function registry (§8) |
| `F_S` | Sum | `F+` | sum of the child functions |
| `F_P` | Product | `F+` | product of the child functions |
| `F_C` | Clamp | `num lo`, `num hi`, `F` | the child value limited to `[lo, hi]` |
| `F_E` | Expression | reserved | A parsable expression string. Reserved until its syntax is decided (open question WP0-3) |

Rules:
* Field names in `F_L` are **scalar** field paths (spec 05 §2), for example `pose.enu.x` or `target.yaw`.
* `F_L` with `n = 0` is a constant `c`.
* A function that reads a field records it, so the executor and the requirement check (spec 03 §5) know which fields a task needs.

Examples:

A proportional velocity command, saturated to ±5 m/s. `F_C` is the parent and `F_1` its child:

```
F_1: F_C -5 5 F_1/
F_1: F_L 2 0.8 -0.8 0 target.x pose.enu.x/
```

The value is `clamp(0.8·target.x − 0.8·pose.enu.x, −5, 5)`. The same command from the registry, with gain 0.8 and limit 5 m/s:

```
F_1: F_X vel_to_target_x 2 0.8 5/
```

## 8. The function registry

```cpp
namespace MRS::Task {
  using RegistryFunction =
      std::function<double(const Environment::Worldview& w, double t, const std::vector<double>& k)>;
  class FunctionRegistry {
  public:
    void Register(std::string key, RegistryFunction f, std::size_t n_coefficients,
                  std::vector<std::string> fields_read);
    bool Has(const std::string& key) const;
  };
}
```

* Keys follow the naming rule in spec 00 §6.
* `n_coefficients` is checked against the `F_X` slot `n` at parse time when a registry is available, and at task load time otherwise.
* `fields_read` lists the worldview fields the function reads, for the requirement check.
* The library registers these UAV functions in WP1:

| Key | Coefficients | Value |
|---|---|---|
| `dist_to_target` | — | 3D distance from `pose.enu` to `target.x/y/z` |
| `dist_to_target_xy` | — | horizontal distance to the target |
| `yaw_to_target` | — | ENU yaw pointing at the target; the current yaw when closer than 1 m horizontally |
| `vel_to_target_x`, `_y`, `_z` | gain, limit | `clamp(gain · (target − pose), −limit, limit)` per axis |
| `layered_x`, `layered_y` | `alt_tol`, `switch_radius` | the target's x or y when `|pose.enu.z − layer.alt|` ≤ `alt_tol` or the horizontal distance to the target is ≤ `switch_radius`; otherwise the current x or y (so the robot climbs in place first) |
| `layered_z` | `switch_radius` | `layer.alt` while the horizontal distance to the target is > `switch_radius`; then `target.z` |

*Note:* the registry replaces the template class `ExternalFunction<T>`. A string key can travel inside a task string, and it does not need a template for each calling class.
