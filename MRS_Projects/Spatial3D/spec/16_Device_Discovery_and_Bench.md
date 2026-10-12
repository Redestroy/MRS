# 16 Device identity, discovery and bench mode

Status: **draft**, spec version `0.2`. Not implemented yet. Extends spec [04](04_Devices_and_Ports.md) (devices and ports), which stays valid for everything this spec does not change. Grammar: [01](01_Protocol_Grammar.md). Background: the device tree brainstorm of 2026-10-12 (project file `design/DeviceTree_Brainstorm_2026-10-12.md`).

## 1. Scope

libmrs is meant for robots whose device trees differ a lot. This spec adds the **base mechanism** that makes trees easy to define, check and test. The convenience tooling (a debug UI, a curated device library, URDF and Webots proto converters, ROS converters) comes later and builds on it.

Every way of building a tree ends in the same text: a `.mrsd` definition plus a `.mrsp` port map, read by the existing tree builder. A tree can be:

1. written by hand (spec 04, unchanged);
2. generated from a simulation model (tooling, later; it emits `.mrsd`);
3. **discovered** from a port scan and the identities the devices report (§7).

JB's decisions (2026-10-12):

| # | Decision | Where |
|---|---|---|
| 1 | Joints that carry data MAY provide the ports of their subtree; the platform is the root provider. The ArduPilot autopilot becomes such a joint | §4 |
| 2 | Nodes inside a complex device get scoped names (`left_wheel.motor`); a short name is still accepted while it is unique | §5 |
| 3 | Port types are an open list of names, so ROS 2 and user-written providers fit | §4.5 |
| 4 | A device reports its **identity**; the description comes from a mapping. Unknown devices are handled explicitly, and a swapped or updated device is handled by reassigning its mapping, which is a small text file | §2, §3, §7.3 |
| 5 | This is a new spec; spec 04 points to it | spec 04 §8 |

Rules that carry over unchanged: a port is never picked because it is free (spec 04 §6); missing or unknown hardware makes a node **unavailable** (a `FAULT`), never a guess (state review, "fail into the pool, not into a guess").

## 2. Identity and descriptor

### 2.1 Identity

An **identity** is a short string a device or a port reports about itself:

```
identity = vendor ":" model [ ":" rev ] [ "@" serial ]
```

* `vendor`, `model` and `rev` are made of lower-case letters, digits, `_`, `-` and `.`. `serial` is any run of characters other than whitespace and `"`.
* Examples: `"cyberbotics:gps"`, `"ardupilot:arducopter:4.5.7"`, `"bosch:bmi088@0x68"`, `"acme:motor_block:2@A13F07"`.
* In records an identity is always a `str` slot, because `:` and `@` are not identifier characters.
* A platform MAY derive identities from what its transport offers. *Note:* sources that exist today include DroneCAN/Cyphal `GetNodeInfo`, MAVLink `HEARTBEAT` and `AUTOPILOT_VERSION`, USB vendor and product ids, and I²C addresses with a `WHO_AM_I` register. Webots reports `cyberbotics:<node type in lower case>`, for example `cyberbotics:gps`.

A **pattern** is an identity where `rev` MAY be `*` or left out and the serial MAY be left out. A pattern matches an identity when vendor and model are equal, the rev is equal or not constrained, and the serial is equal or not constrained. Its **specificity** is the number of constrained parts among rev and serial (0, 1 or 2).

The identity in spec 04 §6.3 (`identity` parameter of a requirement, `PortInfo::identity`) uses this format from version 0.2 on, and matching there is pattern matching. An empty identity means "unknown".

### 2.2 Descriptor

A device MAY also report a **descriptor**: a `.mrsd` fragment made of one `D` record (edge or `D_X`) with its sub-records. It says what the device is and how it is configured, in the same text as a definition file.

* The descriptor's node name is ignored; the node gets its name from the definition or from discovery (§7.2).
* Its type key MUST be a known registry key or part (§6). Its parameters, `K` overrides and `P_R` overrides apply as if they were written in the definition, under the definition's own (§3.3).
* As in spec 04 §4.1, a descriptor can change the limits of a capability the class declares; it cannot add one.

*Note:* only MRS-native smart blocks (a microcontroller programmed to answer) are expected to carry descriptors. The wire query that fetches one is defined per provider; version 0.2 defines none beyond `PortInfo::descriptor` (§4.3).

## 3. Identity maps (`.mrsi`)

An identity map says what a device is, given its identity. This is the mechanism for unknown devices and for devices that are swapped or updated (JB decision 4): when a device changes, one `I_M` line is added or edited and the tree is built again.

### 3.1 Records

New kind `I` (identity), version 0.2:

| Code | Slots | Meaning |
|---|---|---|
| `I_M` | `str pattern`, `id type`, `int n`, `n × (id key, value)` | Devices whose identity matches `pattern` are of registry key or part `type` (§6), with these parameters |
| `I_U` | `id provider`, `str address`, `id port_type`, `str identity`, `[str descriptor]` | Report: a scanned port whose identity no map resolves (§7.2) |
| `I_R` | `id result`, `id node`, `str expected`, `str found` | Report: one finding of the boot check (§7.3) |

* `type` MAY be `ignore`: the device is known and deliberately not used. Discovery skips it and the check reports nothing for it.
* `I_M` parameters MAY include `req` (id): the port requirement of the class that the scanned port fills, needed only when the class has more than one requirement of that port type.
* `I_M` parameters MAY include `name` (id): the node name discovery gives the device (§7.2).

Example identity map for a robot:

```
@: MRS 0.2/
# The GNSS module was replaced by a newer revision; its rev is not in the library yet.
I: I_M "ublox:neo-m9n:*" gnss.serial 1 rate_hz 10/

I: I_M "acme:motor_block:2" part.motor_block 1 motor.max_speed 520/

I: I_M "cyberbotics:display" ignore 0/
```

### 3.2 Sources and precedence

An identity is resolved against these sources, in this order. The first source with a match wins:

1. the robot's own identity map (`<robot>.mrsi`, next to its `.mrsd` and `.mrsp`): the user's override;
2. the descriptor the device reported, if any;
3. the identity maps in the device library (§6.3), in search-path order;
4. the identities the registry's classes declare (`DeviceRegistry::Register<T>(key, defaults, identities)`).

Within one source the most specific matching pattern wins. Two matching patterns of equal specificity in one source, with different `type` or parameters, are a build error.

*Note:* the robot's own map comes first so that the user always has the last word, which is what makes a swap fixable with one line. The device's descriptor comes before the library because it is the newest information about that unit.

### 3.3 Parameter layering

When a node is created, its parameters are merged in this order, later entries replacing earlier ones of the same key (except repeatable keys, which are replaced as a group):

1. registry defaults of the type key (spec 04 §5);
2. the winning identity source: the `I_M` parameters, or the descriptor's parameters;
3. the definition file's parameters for the node.

The definition file therefore always wins, the resolved identity fills in what it leaves open, and the registry gives the rest.

## 4. Port providers

### 4.1 Providers

A **port provider** scans and opens ports. The platform (spec 04 `IPlatform`) is the **root provider**, named `platform`. A joint MAY be a provider for its subtree:

```cpp
namespace MRS::Port {
  class IPortProvider {
  public:
    virtual std::vector<PortInfo> Scan() = 0;
    virtual std::unique_ptr<Port> Open(const PortAssignment&) = 0;
    virtual std::vector<std::string> PortTypes() const = 0;   // the types it offers, §4.5
    virtual void Update(double t) {}                          // once per control tick, before sampling
  };
  class IPlatform : public IPortProvider { /* Time(), Step() as in spec 04 */ };
}
```

* A joint becomes a provider with the parameter `provider <type key>`, a registry key of a class derived from `ProviderJoint` (which is a `JointNode` and an `IPortProvider`). Its interface kind MUST carry data (`bus`, `mech_bus`, `mech_bus_elec`); otherwise it is a build error. Joints without `provider` stay passive, as in spec 04.
* A provider joint has its own port requirements towards **its** provider (for example a UDP link to the autopilot) and its own parameters. Those ports are resolved like any node's.
* Time and stepping stay with the platform. A provider joint does its per-tick work in `Update(t)`, which the robot calls on every available provider, root first, in tree order, before sensors are sampled.

### 4.2 Resolution

Each port requirement of a node is resolved by the **nearest provider above the node** that offers the requirement's port type. The search walks up from the node's parent to the platform. If no provider offers the type, the requirement is a build error.

* The sources of spec 04 §6.3 (port map, explicit address, identity scan) apply unchanged, using the chosen provider's scan and `Open`.
* An address is local to its provider. Two providers MAY both have an address `node12`.
* A `P_A` entry MAY name the provider with the parameter `via <joint name>` (`platform` for the root). If given, it MUST equal the provider the rule above picks; otherwise it is a build error. `WritePortMap` writes `via` for every entry not on the platform.
* The exclusive-port rule of spec 04 §6.3 applies per provider.

### 4.3 Scanning

A scan is recursive: the platform scans; every provider joint that is built and available scans its own ports. `PortInfo` gains two fields:

| Field | Meaning |
|---|---|
| `provider` | The name of the provider that reported the port |
| `descriptor` | The device's descriptor text (§2.2), empty if none |

### 4.4 Availability

A node depends on every provider between it and the platform. If a provider joint is unavailable (a port that did not open, a failed `Start`, or an unavailable provider above it), every node it provides for is unavailable with the reason `provider <joint> is unavailable`, through the existing `DependsOn()` propagation (spec 04 §6.3).

### 4.5 Port types

Port types are names, not a closed enum (JB decision 3).

* A name is upper-case letters and digits, starting with a letter, with no `_` (so it never reads as a code): `UART`, `MAVLINK`, `ROS2`.
* The names of spec 04 §6.1 stay built in: `GPIO PWM ADC UART I2C SPI CAN UDP TCP MAVLINK SIM`.
* Any other name is valid in a file when some provider in the tree offers it (`PortTypes()`); otherwise the requirement is a build error at port assignment.

*Note:* this is what lets a ROS 2 provider offer port type `ROS2` with topic names as addresses and message types as identities, and lets a user write a provider in Python through the C API.

### 4.6 The ArduPilot autopilot as a provider

The ArduPilot platform of spec 14 becomes a provider joint, `autopilot.mavlink`, with one requirement (`link`, port type `UDP` or `UART`) on the platform. Its scan reports the same MAVLink ports as today (spec 14 §3). The robot's platform is then the host (the companion computer, or a host platform for SITL), so a robot can carry other providers next to the autopilot.

Until that change is implemented, `ArduPilotPlatform` remains a root platform and `quad_ardupilot.mrsd` stays valid as written. The clock modes of spec 14 (autopilot boot time, wall clock) move to the host platform when the split is made.

## 5. Scoped node names

JB decision 2. Names are action targets (spec 02 §5) and appear in task text, port maps and the self model.

* A node that has a `D_X` ancestor has a **full name** made of the `D_X` names from the outermost down, then its own local name, joined by `.`: the motor `m_fl` of complex device `quadrotor` is `quadrotor.m_fl`; a motor inside part `arm` inside part `wrist` is `arm.wrist.motor`. Joints do not add a segment.
* A node with no `D_X` ancestor has its local name as its full name, as in version 0.1.
* A **local name** MUST NOT contain `.`. Full names MUST be unique in the tree.
* **References** (action targets, `P_A` and `P_R` node names, `req`/`node` slots, `DependsOn`) accept a full name, or any `.`-suffix of one (`m_fl`, `wrist.motor`) that matches exactly one node. A suffix that matches several nodes is ambiguous: a build error in a file, `REJECTED` for an action.
* Writers (tree writer, `WritePortMap`, self model, reports) always write full names.

*Note:* with this rule, existing task and port map files that say `m_fl` keep working while the tree has one quadrotor, and a tree MAY now hold two complex devices of the same type.

## 6. Parts and the device library

### 6.1 Text-defined complex devices

The registry key `group.default` is a complex device that creates no virtual children. Its explicit `D` children (spec 04 §2.1) are its contents. It has no capabilities and no ports of its own.

### 6.2 Parts

A **part** is a reusable branch in a file `<part name>.mrsd` whose single top-level record is a `D_X` (not a `D_H`) with its sub-records. It is typically a `group.default` with children, for example a motor block made of a driver, a motor and an encoder.

* A definition includes a part with the type key `part.<part name>`, on a `D_X` record: `D_1: D_X left_wheel part.motor_block 1 motor.max_speed 300/`.
* The included node takes the part root's type key, parameters and children. The including record's name replaces the root's name, so the part's nodes get full names under it (§5).
* A parameter whose key has the form `<local>.<key>`, where `<local>` is the local name of a node in the part, sets that parameter on that node (`motor.max_speed`). Any other key applies to the part's root. A `<local>` that names no node in the part is a build error.
* The including record's `K` and `P_R` sub-records apply to the root; its `D` children follow the part's children.
* `part` is reserved as a device name in type keys.

### 6.3 Library

The device library is a list of folders (the **search path**), set by the application (`MRS_LIBRARY` environment variable, or the C API). Each folder MAY contain:

* `parts/*.mrsd`: parts, named by file stem;
* `*.mrsi`: identity maps (§3).

A part name found in two folders is taken from the first one on the search path. The curated contents of a standard library are tooling and not part of this spec.

## 7. Tree writer, discovery and the boot check

### 7.1 Tree writer

`DeviceTree::ToDocument()` writes the **definition** of a built tree in canonical form (spec 01 §3.5): virtual nodes are left out (their complex device recreates them), part includes are written as includes with the parameters that were given, and parameters are those of the definition and the descriptor, not registry defaults. Building the written document gives the same tree.

### 7.2 Discovery

```cpp
DiscoveryReport Discover(const Protocol::Record& head, const DeviceRegistry&, const Library&,
                         const IdentityMap* robot_map, Port::IPlatform&);
struct DiscoveryReport { Protocol::Document mrsd, mrsp, unknown; };  // unknown: I_U records
```

The head cannot be discovered, so the caller gives its `D_H` record (children ignored). Then:

1. The platform is scanned. Each scanned port's identity is resolved (§3.2). A port with an empty or unresolved identity becomes an `I_U` record; an `ignore` match is skipped.
2. Each resolved port becomes one node of the resolved type, with the resolved parameters. If the class has several requirements of the port's type, `req` in the mapping says which one the port fills; the other requirements are left to the normal rules when the tree is built.
3. The node's name is the mapping's `name` parameter, else the port's address in lower case with spaces and invalid characters turned into `_` if that is a valid unused identifier, else the model followed by `_1`, `_2`, ….
4. A node that resolves to a provider joint class becomes a `D_J` with `provider` set, is opened, and its own scan is processed the same way, its nodes becoming its children.
5. The nodes found on each provider are placed under one `D_J` of interface `mech_bus` (named `base` for the platform), and those joints under the head.
6. The `mrsp` holds a `P_A` per filled requirement, with the scanned port and the parameter `identity "<identity as scanned>"`.

The report is a **proposal**. It is never built automatically: the user checks it, completes the unknowns (usually by adding `I_M` lines to the robot's `.mrsi`), runs discovery again if needed, and saves the result. This is the same flow as first-time port setup (spec 04 §6.4).

### 7.3 Boot check and swapped devices

When a `P_A` entry carries an `identity` parameter, `BuildRobot` compares it with what the provider now reports at that address. Each difference is an `I_R` finding:

| Result | When | Effect |
|---|---|---|
| `missing` | the address is not in the scan | node unavailable (`FAULT`), as in spec 04 |
| `swapped` | same vendor, model and rev; another serial | node available; the new identity is reported |
| `updated` | another model or rev, and the new identity resolves to the node's type key | node available; the resolved parameters of the **new** identity are used (§3.3), so an updated device's information takes effect |
| `changed` | the new identity resolves to another type key, or to nothing | node unavailable (`FAULT`) until the mapping or the definition is changed |
| `extra` | a scanned port on a provider of the tree whose identity resolves but which no node uses | information only |

* Nothing is substituted silently: a device that is now something else leaves its node unavailable, and the task layer sees the capability as missing.
* The runtime never rewrites the robot's files. The findings are returned with the robot (`Robot::check`, a list of `I_R`), and `WritePortMap` writes the new identities when the user decides to save.
* Fixing a `changed` device is one line: an `I_M` in the robot's `.mrsi` that maps the new identity to the node's type key (with its parameters), or a change of the node's type key in the `.mrsd`.

Example check report:

```
@: MRS 0.2/
I: I_R updated gnss "ublox:neo-m8n:3.01" "ublox:neo-m9n:4.04"/

I: I_R changed left_wheel.motor "acme:motor_block:2" "acme:motor_block:3"/
```

## 8. Bench mode

Bench mode lets a user build part of a robot and command its devices directly, before the robot is assembled or before a task exists.

### 8.1 Building

`BuildRobot` takes a flag `bench`. A bench robot is built like any other (tree, ports, start, availability, blocks, self model), but it MUST NOT be given to a `RobotController`, and a robot built without the flag does not accept bench commands. A definition MAY be as small as a head with one joint and one device.

### 8.2 Commands

| Command | Behaviour |
|---|---|
| `Apply(node, action)` | One `A` record (with sub-records) sent to the named node through the actuator block's routing (spec 04 §4.2). Returns the action status |
| `Sample(node)` | The node's new views as `V` records |
| `Nodes()`, `Faults()`, `Self()`, `Check()` | The tree with full names, the faults, the self model, the boot check findings |
| `OpenRaw(provider, port_type, address, params)` | Opens a port that no node holds, for testing wiring before a device class exists. Refused for a port a node already holds exclusively |
| `Read`, `Write`, `Send`, `Receive` on a raw port | As `Port` (spec 04) |
| `Stop()` | Every actuator to its safe state (§8.3) |
| `Step()` | One platform step; providers' `Update`, sensor sampling and actuator `Update` run as in a normal tick |

### 8.3 Safety

* An action argument outside a `min` or `max` limit of the node's `K_A` is `REJECTED`.
* The **motion codes** `A_MOT A_TO A_LD A_HD A_PXY A_PZY A_VXY A_VZY A_GMB` are `REJECTED` unless the bench was built with them in its `allow` list.
* **Dead man:** if an actuator receives no bench command for `bench_timeout` seconds (default 0.5), it is put in its safe state. `Actuator::Safe()` is new; by default it does nothing. Motors go to zero; the flight control unit holds or lands as it does on `setpoint_timeout`.
* The safety supervisor and the task executor do not run in bench mode. *Note:* that is why bench mode is a build flag and not a switch on a running robot.

## 9. C API touchpoints

*Note (non-normative).* The C API and the Python bindings are the next step. This spec is shaped so that they can be thin:

* Everything crosses the boundary as protocol text: `.mrsd`, `.mrsp`, `.mrsi`, descriptors, `A`, `V`, `I_U` and `I_R` records. The ABI stays small.
* Opaque handles for the registry, the library, a platform or provider, and a robot. Nodes are addressed by name (§5). The tree is fixed after build, so names stay valid for the robot's lifetime.
* User-written **device classes** are a table of callbacks (`configure`, `start`, `sample`, `apply`, `update`, `safe`) registered under a type key, with optional identities.
* User-written **providers** are a table of callbacks (`scan`, `open`, `read`, `write`, `send`, `receive`, `update`) with their port type names (§4.5).
* `BuildError` becomes an error code and message; faults and check findings are lists of records.
* The bench commands of §8.2 are the first slice of the C API.

## 10. Protocol changes

* Kind `I` with codes `I_M`, `I_U`, `I_R` (§3.1). A version 0.1 parser rejects them as unknown codes; files that use them SHOULD start with `@: MRS 0.2/`.
* New file extensions: `.mrsi` (identity map) and `.mrsr` (report: `I_U` and `I_R` records).
* `D_J` parameter `provider` (§4.1), `P_A` parameter `via` (§4.2), `P_A` parameter `identity` (§7.2).
* Open port type names (§4.5).
* Type keys `group.default` and `part.<name>` (§6).

## 11. Implementation order

*Note (non-normative).* A suggested order, each step with its tests:

1. Identity format and pattern matching; kind `I` in the parser and writer; `.mrsi` loading; registry identities (§2, §3).
2. Scoped names and suffix resolution (§5), with the existing examples still passing.
3. `group.default`, parts and the library search path (§6).
4. Tree writer (§7.1).
5. Providers, open port types, recursive scan, `via` (§4); then the ArduPilot split (§4.6).
6. Discovery and the boot check (§7.2, §7.3).
7. Bench mode (§8).
8. The C API, starting with the bench slice (§9).

## 12. Defaults chosen here, for review

These were not asked explicitly and follow from the rules above. Each can be changed.

1. A device that now resolves to another type key is not substituted: its node is unavailable until the mapping is changed (§7.3).
2. A device's own descriptor ranks above the library and below the robot's own map (§3.2).
3. The runtime reports differences but never rewrites the robot's files (§7.3).
4. Parts take parameters only through `<local>.<key>` overrides; there is no templating (§6.2).
5. Bench mode rejects out-of-limit and motion actions unless allowed, and stops actuators after 0.5 s without a command (§8.3).
