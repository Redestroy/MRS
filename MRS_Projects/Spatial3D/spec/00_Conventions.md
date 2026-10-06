# 00 Conventions

Status: **draft for WP0**, spec version `0.1`.

## 1. Normative wording

* **MUST** / **MUST NOT**: an absolute requirement. A parser, writer or runtime that breaks one is wrong.
* **SHOULD** / **SHOULD NOT**: the default. Deviations need a documented reason.
* **MAY**: optional.
* **Reserved**: the name or code is taken but its meaning is not defined yet. A parser MUST reject reserved codes until a later spec version defines them.

Text marked *Note* explains a rule and is not normative.

## 2. Units

All quantities use SI units unless the field or argument name says otherwise.

| Quantity | Unit | Notes |
|---|---|---|
| Length, position | metre (m) | |
| Time, duration | second (s) | |
| Speed | m/s | |
| Acceleration | m/s² | |
| Angle | radian (rad) | Degrees are used only for geodetic latitude and longitude |
| Angular rate | rad/s | |
| Energy | watt-hour (Wh) | Battery energy |
| Power | watt (W) | |
| Voltage | volt (V) | |
| Mass | kilogram (kg) | |
| Fraction | 0–1 | For example battery remaining. Percent is not used |
| Latitude, longitude | degree, WGS-84 | Altitude AMSL in metres |

## 3. Frames

* **Local frame: ENU** (x = East, y = North, z = Up). The origin is the mission's geo reference point (spec 06 §6). Every position in a task, condition, view or message is in ENU unless the code says geodetic.
* Geodetic to ENU conversion is exact: WGS-84 geodetic to ECEF, then rotation into the ENU frame at the geo reference point. *(Chosen in WP1.)*
* **Body frame: FLU** (x = Forward, y = Left, z = Up), right-handed.
* **Yaw:** the ENU yaw. 0 points East, and counter-clockwise is positive. Range (−π, π].
* **Heading:** navigation heading. 0 points North, and clockwise is positive. Range [0, 2π). `heading = wrap_2pi(π/2 − yaw)`.
* **Attitude:** roll, pitch, yaw, applied in Z-Y-X order (yaw, then pitch, then roll), body to ENU.
* Platform frames (Webots, NED in ArduPilot) MUST be converted at the port or processor boundary. Layers 2 and 3 only see ENU.

## 4. Time

* Library time is a `double` in seconds, read from the platform `Clock`. In simulation this is simulation time.
* **Mission time** `t` is the time since the mission header's `t0` (spec 06 §6). All stamps in messages, journals and logs are mission time, in seconds with at least millisecond resolution.
* Wall time MUST NOT be used for decisions in simulation.

## 5. Numbers and text

* Decimal point is `.`. No thousands separators.
* Text-protocol numbers follow spec 01 §2.3. `nan` and `inf` are not valid.
* Booleans in the text protocol are `T` and `F`.
* Identifiers are ASCII and case-sensitive.

## 6. Naming

| Thing | Style | Example |
|---|---|---|
| C++ namespaces | `MRS::PascalCase` | `MRS::Environment` |
| C++ classes, structs, enums | `PascalCase` | `TaskExecutor` |
| C++ methods | `PascalCase` | `GetNextCommand()` (keeps the existing library style) |
| C++ enum values | `UPPER_SNAKE` | `TaskState::IN_PROGRESS` |
| Worldview field names | lower-case, dot-separated path | `pose.enu`, `alt.agl` |
| Registry keys (functions, devices, behaviours) | lower-case, dot or underscore separated | `fly_to_vel`, `gnss.webots`, `flyto.layered` |
| Protocol object codes | as listed in spec 01 §4 | `T_A`, `C_P3`, `A_PXY` |

## 7. Versioning

* The spec has a version `MAJOR.MINOR`. A MINOR bump only adds codes or optional slots. A MAJOR bump may change existing meaning.
* Every top-level file in the text protocol MAY start with a version record `@: MRS 0.1/` (spec 01 §5). A parser MUST reject a file whose MAJOR version it does not support.
* Messages carry the protocol version in the envelope (spec 06 §2).

## 8. Compatibility with earlier formats

* The **core library format** (records ending in `/`, depth-first sub-objects, as in `MRS_Algorithms_Testing/main.cpp`) is the base of this spec. Its codes keep their meaning where this spec lists them.
* The **2021 E-puck format** (`T: 0 0 T_A 1 /C_S C_P ...`) is a different dialect. It is **not** accepted by the new parser. A converter script in `Tools/` will translate the 25 task sets (WP1).
