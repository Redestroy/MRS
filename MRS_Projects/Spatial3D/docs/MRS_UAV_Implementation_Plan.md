# MRS UAV Implementation Plan: 3D Spatial Task Allocation

**Date:** 2026-10-04. This is a plan only; no code was written and no earlier file was changed.
**Guide:** JB's implementation note (thread message of 2026-10-04: layers 0–3 plus the worldview side layer, device tree, robot controller, task issuer), and the decisions recorded in `Thesis_vs_Library_Analysis.md` §8 and `MRS_Framework_Total_Analysis.md` §12.
**Sources read for this plan:** `MRS_Framework_Total_Analysis.md` (entry point), `Thesis_vs_Library_Analysis.md` §8, `MRS-RTA_Integration_Evaluation.md` §5–6, the core headers in `Inputs\MRSlib\MRS\MRS\Include`, `MRS_Algorithms_cpp\Include\MRS_Layer.hpp`, the E-puck app in `Inputs\controllers`, and the paper *Dynamic Spatial Task Allocation in Multi-Robot Systems* (Oct 2021, `Inputs\Dynamic Spatial Task Allocation in Multi-Robot Systems.pdf`).

Class names in `code` that exist today are marked **(exists)**. Everything else is a proposed name. Text-protocol strings in this document are illustrative; the grammar step (WP0) fixes the real letters.

---

## 1. Summary

**Goal.** Build a group of flight-capable UAVs (quadcopters first) on the MRS framework that allocates and executes 3D spatial tasks, and use it to answer one question:

> **Does a group of N UAVs complete a set of spatial tasks faster than one UAV that flies a planned mission over the same set?**

**What the library owns and what the user supplies.** Almost everything lives in the library. The user supplies only:

1. **Tasks**, as text-protocol strings or through a task generator fed with operator-domain input (points, polygons, paths in geographic coordinates).
2. **The hardware interface**: a platform implementation of the port layer (`IPlatform`) and a communication transport (`ITransport`).

The library ships both of these for Webots, and the ArduPilot platform as the second implementation. It also ships the quadcopter device definitions, worldview processors, the UAV behaviour library, the allocators and the evaluation harness.

**Shape of the solution, by layer.**

| Layer | What gets built | Main existing objects it expands |
|---|---|---|
| 0 Port | `Port`, `PortManager`, `IPlatform`; Webots and ArduPilot (MAVLink) platforms; automatic port assignment from device metadata | `Controller` (exists) |
| 1 Device | A real `DeviceTree` (head, joint, edge and complex nodes) with node metadata and capabilities; a device registry for "implement once, reuse"; block generation by traversal; the `Quadrotor` complex node with 4 motor actuators, sensors, LEDs, battery as storage and a radio | `Device`, `DeviceTree`, `Sensor`, `Actuator`, `SensorBlock`, `ActuatorBlock`, `CommunicationDevice`, `FeedbackPair` (all exist) |
| W Worldview | A view router with plug-and-play processors that turn views into world fields and semantic objects each tick; time series; geo reference; worldview requirements so robots can tell what they cannot do | `Worldview`, `View`, `ViewType::VIEW_POSITION_3D` etc. (exist) |
| 2 Robot | `RobotController` in the head node: device-tree initialisation, the agent loop with interrupts, task executor, behaviour library with parametric tasks, resource manager, safety supervisor; behaviours Liftoff, Land, FlyTo (several variants), Hover, FlashLED, ReturnHome | `Robot`, `Agent`, `Task`, `ATask`, `Behaviour`, `BehaviourLibrary`, `ParametricAction`, `ParametricATask`, `MathFunction`, `Condition` family (exist) |
| 3 MRS | `MRSLayer` hosting a generic `IAllocator`; MRS-RTA first, then CBBA and a mission-planner baseline, then MRS-STA with tree-task decomposition, then LDTA²; message envelope with broadcast and P2P; task issuer and task generators | `Layer` (exists), E-puck `TaskPriorityList`, `CBAA_Manager`, `MessageTranslator` |

**Order.** The evaluation question can be answered after work package 6 (§11): one Webots world, N simulated quadcopters running MRS-RTA, and a single quadcopter running a planned route, on the same task sets. Tree tasks split across robots (JB's end goal for the MRS layer) come in WP7–8. ArduPilot hardware comes last, after the same code has run against ArduPilot SITL.

**Why 3D helps the 2D result.** The 2021 paper reports that 14 of 25 CBAA runs and 1 MRS-RTA run stalled because robots blocked each other near their start points. Flying robots can be separated by altitude, so the plan includes altitude-layered flight as a standard FlyTo variant (§7.4). That should remove most of the deadlock problem and make the algorithm comparison cleaner than in 2D.

---

## 2. Requirements taken from the implementation note

Each requirement is numbered so later sections can refer to it.

| # | Requirement (JB) | Where it is met |
|---|---|---|
| R1 | Four layers plus a worldview side layer | §3 |
| R2 | Worldview gets views as inputs, processes them into semantic objects, and turns view information into world information every timestep | §6.1–6.3 |
| R3 | For spatial tasks: time tracking of view information; position, velocity and acceleration tracking; rotation and orientation | §6.4 |
| R4 | Robot-dependent extras: time series, maps, sync with other robots' worldviews | §6.5–6.6 |
| R5 | A task may need a minimum set of worldview fields; robots lacking them dump the task and focus on what they can do | §6.7, §7.7, §8.3 |
| R6 | Worldview processing is plug and play: a processor interested in a view type gets it and writes results back | §6.2 |
| R7 | Device layer as a `DeviceTree` with head, joint and edge nodes representing physical connectivity | §5.1–5.3 |
| R8 | Sensor and actuator blocks generated by traversal; implement once, reuse device definitions | §5.4–5.5 |
| R9 | Optional layer 0 (port layer) that manages protocols and GPIOs and assigns them to device definitions automatically | §4 |
| R10 | Head node handles task processing and tree updates and contains the robot controller | §5.2, §7.1 |
| R11 | Edge nodes: sensors, actuators, communication devices and storage, with capabilities that translate to actions, views, messages or capacity | §5.2 |
| R12 | Joint nodes are links with a defined interface and connection rules | §5.2 |
| R13 | Complex devices: one physical block represented by a branch of virtual nodes | §5.3 |
| R14 | Nodes carry enough metadata that the head can aggregate a model of self | §5.6 |
| R15 | Robot controller: initialise the device tree, run the agent loop, update the world model, parse and send messages, allocate resources, assign actuator actions, optionally learn | §7.1–7.2 |
| R16 | The agent loop is robust to events such as an urgent message or a fault | §7.3 |
| R17 | Layer 3 aggregates tasks and selects the best task for the robot | §8 |
| R18 | Layer 3 communicates with the operator (task issuer), including task generation from operator-domain information (geodata, shapes) | §9 |
| R19 | Port layer: Webots simulation and an undefined hardware platform, probably ArduPilot | §4.3–4.4 |
| R20 | Device layer: the robot is probably one complex node with sensors and 4 motor actuators; battery as storage representing energy; full device tree as a proof of concept | §5.3 |
| R21 | Robot layer: device-tree initialisation, tasks, a behaviour library with parametric tasks; behaviours land/liftoff, fly to point in various ways, flash LEDs | §7 |
| R22 | MRS layer: the noted task-selection algorithms, MRS-RTA first, then the more complex tasks; end goal is a tree task split into its base tasks and shared between robots | §8 |
| R23 | Worldview minimum: timing, altitude, heading, GNSS position. Optional: object detections, drone-relative position sensor | §6.4–6.5 |
| R24 | Layers 2 and 3 assume a generic flight-capable machine | §5.7 |
| R25 | Core evaluation: group vs single UAV with mission planning on spatial tasks | §10 |
| R26 | Users define only tasks and the hardware/communication interface | §12 |
| R27 | Expand the current objects; object-oriented, agent paradigm matching MRS | §13 |

Earlier decisions that bind this plan: MRS-RTA first behind a generic interface, then MRS-STA, LDTA² and CBBA; `Priority = base × size_est(w, t)`; broadcast and P2P; a behaviour is a task type mapped to conditions; ParametricAction binds worldview fields through a function string or a named `f(w, t)` registry; an action has one 64-bit splittable argument and combined actuators take a map of actions; a role is a robot-type requirement; SCA is a separate later module; TTA is team task allocation.

---

## 3. Architecture

### 3.1 Layers for the UAV system

```
 Operator domain (geodata, shapes, mission files)
        │  TaskGenerator → task strings
 ┌──────▼──────────────────────────────────────────────────────────────┐
 │ 3  MRS layer        MRSLayer: TaskPool, PeerTable, IAllocator        │
 │                     (MRS-RTA → CBBA, Planner baseline → MRS-STA →   │
 │                     LDTA², TTA), ITaskDecomposer, size_est          │
 └──────┬───────────────────────────────────────────▲──────────────────┘
        │ Assign(task) / Preempt                     │ task state, capability profile
 ┌──────▼───────────────────────────────────────────┴──────────────────┐   ┌──────────────────────┐
 │ 2  Robot layer      RobotController (lives in the head node):        │   │ W  Worldview          │
 │                     agent loop, EventQueue, TaskExecutor,            │◄─►│ ViewRouter →          │
 │                     BehaviourLibrary, FunctionRegistry,              │   │ IViewProcessor chain  │
 │                     ResourceManager, SafetySupervisor, Messenger     │   │ → WorldFields,        │
 └──────┬───────────────────────────────────────────▲──────────────────┘   │ SemanticObjects,      │
        │ Action / ActionMap (generic flight set)    │ Views               │ TimeSeries, Clock,    │
 ┌──────▼───────────────────────────────────────────┴──────────────────┐   │ GeoReference          │
 │ 1  Device layer     DeviceTree: HeadNode, JointNode, edge nodes,     │──►│                       │
 │                     ComplexDevice (Quadrotor branch), SelfModel,     │   └──────────────────────┘
 │                     blocks built by traversal, DeviceRegistry        │
 └──────┬──────────────────────────────────────────────────────────────┘
        │ read/write on assigned ports
 ┌──────▼──────────────────────────────────────────────────────────────┐
 │ 0  Port layer       PortManager, Port; IPlatform:                    │
 │                     WebotsPlatform | ArduPilotPlatform (MAVLink)     │
 └─────────────────────────────────────────────────────────────────────┘
        firmware / simulator
```

The boundaries are the ones JB named: the bottom boundary is the firmware or simulator (layer 0), and the top boundary is the operator who issues tasks (layer 3).

### 3.2 Modules and namespaces

The rewrite recommended in the assessment (CMake, C++17, no Boost, one C API) stays. The namespaces that exist today are kept and two are added.

```
mrs/
  core/
    port/        MRS::Port          (new)  Port, PortType, PortRequirement, PortManager, IPlatform
    device/      MRS::Device        Device, DeviceNode, HeadNode, JointNode, ComplexDevice,
                                    Sensor, Actuator, CommunicationDevice, StorageDevice,
                                    Capability, DeviceDescriptor, DeviceTree, DeviceRegistry,
                                    SensorBlock, ActuatorBlock, CommBlock, StorageBlock, SelfModel,
                                    Action, ActionMap, View + view types
    world/       MRS::Environment   Worldview, WorldField, TimeSeries, SemanticObject, ViewRouter,
                                    IViewProcessor, Clock, GeoReference, WorldviewRequirements
    task/        MRS::Task          Task, ATask, ParametricATask, Behaviour, ComplexTask family,
                                    Condition family, BehaviourLibrary, FunctionRegistry,
                                    ParameterBinding, TaskRequirements
    robot/       MRS::Task          Agent, Robot, RobotController, TaskExecutor, EventQueue,
                                    ResourceManager, SafetySupervisor, CapabilityProfile
    comm/        MRS::Comm          (new)  Message, ITransport, MessageCodec, Messenger
    protocol/    MRS::Protocol      grammar, parser, writer (replaces TreeTokenizer/SIMPLE_MRS_PARSER)
  algorithms/    MRS::Algorithms    MRSLayer, TaskPool, PeerTable, IAllocator, ISizeEstimator,
                                    RtaAllocator, CbbaAllocator, PlannerAllocator, StaAllocator,
                                    Ldta2Allocator, ITaskDecomposer, TaskIssuer, task generators
  platforms/
    webots/      WebotsPlatform, WebotsRadioTransport, Webots device ports
    ardupilot/   ArduPilotPlatform, MavlinkPort, UdpTransport (MAVLink headers only here)
  devices/
    uav/         Quadrotor, RotorMotor, FlightControlUnit, Gnss, Imu, Barometer, Compass,
                 LedArray, Battery, Radio, RelativePositionSensor, Camera (optional)
  behaviours/
    uav.mrsb     the UAV behaviour library as text
  experiments/
    uav_spatial/ Webots worlds, task sets, supervisor, batch runner, analysis
  capi/, bindings/   unchanged from the assessment plan
```

Dependencies point downward only: `algorithms` uses `core`; `platforms` and `devices` use `core`; nothing in `core` knows Webots or MAVLink.

### 3.3 One tick, end to end

This is the data flow that every later section fills in:

1. **Port → Device.** `PortManager::Poll()` reads the platform. Each sensor's `Update(dt)` (exists) produces a `View` from its port.
2. **Device → Worldview.** The sensor block hands views to the `ViewRouter`. Interested processors run and write `WorldField`s and `SemanticObject`s.
3. **Comm → Worldview / MRS.** Incoming messages are decoded. Peer state becomes a view (`VIEW_PEER_STATE`) and goes through the same router; task and allocation messages go to the MRS layer.
4. **Events.** Faults, urgent messages and safety triggers are pushed into the `EventQueue` and handled before normal task work (§7.3).
5. **MRS layer.** `IAllocator::Select(w, t)` returns the task this robot should pursue, or keeps the current one.
6. **Robot layer.** `TaskExecutor::Step` asks the active task for its next command. If the start condition is unmet, it runs the behaviour that the behaviour library maps to that condition. Parametric actions are recalculated from the worldview first.
7. **Device dispatch.** The resulting `Action` or `ActionMap` is routed through the actuator block by action type to the actuator that declares that capability (`FlightControlUnit`, `LedArray`).
8. **Port write and comm out.** Actuators write to their ports; the messenger flushes the allocator's outbox.
9. **Bookkeeping.** The resource manager updates energy, and the logger records the tick for the evaluation.

---

## 4. Layer 0: the port layer

### 4.1 Purpose

The port layer is the only place that knows how bytes or simulator calls reach a device. A device definition says what kind of port it needs ("a UART at 115200", "a PWM output", "a Webots device named `gps`", "MAVLink message `GLOBAL_POSITION_INT`"). The port manager matches requirements to what the platform offers and hands each device its port. That is what makes a device definition reusable across robots (R8, R9).

### 4.2 Objects

| Object | Responsibility | Built from |
|---|---|---|
| `Port` | Abstract handle: `Open`, `Close`, `Read`, `Write`, `IsOpen`, `GetType`, `GetAddress`. Byte-oriented ports and typed ports (simulator handles, MAVLink channels) share this base; typed ports add typed accessors in subclasses | Generalises `Controller` (exists), whose `SendCommand(template, args)` survives as `ProtocolPort`, a helper for command-template protocols |
| `PortType` | `GPIO`, `PWM`, `ADC`, `UART`, `I2C`, `SPI`, `CAN`, `UDP`, `TCP`, `MAVLINK`, `SIM_DEVICE` | Extends `PROTOCOL_TYPE` (exists, used by `Controller`) |
| `PortRequirement` | What a device needs: type, optional address or name, parameters (baud, bus address, message id), exclusive or shared | New; lives in the device descriptor (§5.6) |
| `PortManager` | Asks the platform for available ports, matches each device's requirements, detects conflicts (two devices on one exclusive pin), reports unassigned devices, polls ports each tick | New |
| `IPlatform` | **User-supplied hardware interface.** Enumerates ports, opens them, gives the clock and the time step, and says whether the platform has a flight controller of its own | New |

Assignment rule: exact name match first (`SIM_DEVICE "gps"`), then type and address, then first free port of the right type. A device that gets no port is marked unavailable, so its capabilities drop out of the self model (§5.6) and the robot does not take tasks that need it (R5).

### 4.3 Webots platform (first implementation)

* `WebotsPlatform` wraps `webots::Robot`. It enumerates devices with `getNumberOfDevices()` / `getDeviceByIndex()` and offers each as a `SIM_DEVICE` port named after the Webots device, so device descriptors can match on name.
* Clock: simulation time (`getTime()`), step: `getBasicTimeStep()`. All library timing goes through the `Clock` interface (§6.2), so nothing uses wall time in simulation.
* Robot model: Webots' bundled DJI Mavic 2 Pro model is the natural first airframe. From memory it exposes four propeller motors, GPS, inertial unit, compass, gyro, a camera with gimbal motors, and front LEDs. I have not checked it against R2025a; WP2 confirms the device names. Battery can come from the Webots `Robot.battery` field and battery sensor, so energy is simulated rather than invented (also to confirm in WP2).
* No flight controller exists in Webots, so for this platform the library's `FlightControlUnit` (§5.3) runs the attitude and position loops and mixes to the four motors. Webots' own Mavic example controller is a reference for the gains.
* Communication: `WebotsRadioTransport` over an Emitter/Receiver pair, with range and channel settings, implementing broadcast natively and P2P by recipient filtering (§8.6).

### 4.4 ArduPilot platform (second implementation)

* `ArduPilotPlatform` connects over MAVLink: UDP to SITL during development, serial or UDP to a flight controller on hardware. MAVLink's generated C headers are the only new dependency and stay inside `platforms/ardupilot`.
* Ports are typed MAVLink channels: inbound messages (`GLOBAL_POSITION_INT`, `LOCAL_POSITION_NED`, `ATTITUDE`, `VFR_HUD`, `BATTERY_STATUS`, `HEARTBEAT`) and outbound commands (`SET_POSITION_TARGET_LOCAL_NED`, `COMMAND_LONG` for arm, takeoff and land, mode changes). LEDs and other GPIO either use the flight controller's relay or servo outputs or a companion computer's GPIO; both are just ports.
* ArduPilot already stabilises the airframe and fuses its sensors (EKF), so on this platform the four motors are **not** driven by the library. `FlightControlUnit` forwards the generic flight actions as GUIDED-mode setpoints, and the worldview processors pass ArduPilot's estimates through instead of fusing raw sensors (§6.3). The device tree stays the same shape; only the drivers behind the nodes change (§5.7).
* Development path: ArduPilot SITL alone first, then ArduPilot SITL driving a Webots model (ArduPilot ships a Webots SITL integration; which Webots version it supports needs checking), then hardware. This lets the hardware path be tested with the same worlds and task sets.
* Inter-robot communication on hardware is undefined, so it is a user-supplied `ITransport`. The plan ships a `UdpTransport` (Wi-Fi or mesh) as a default.

---

## 5. Layer 1: the device layer as a device tree

### 5.1 Why a tree

The tree represents physical connectivity. Blocks (the flat lists the robot uses) are generated from it, not hand-built. The current `DeviceTree` (exists) has the right idea (`DeviceTopology { LEAF, JOINT, HEAD }`, parent, children, `Insert(device, id, parentId)`), but its insertion is broken and it has no iterator (its own TODO says so). The older `MRSbase` tree is commented out. The plan replaces its internals and keeps its interface names.

### 5.2 Node kinds

| Node | Role (JB) | Proposed object | Existing object it extends |
|---|---|---|---|
| Head | Handles task processing and tree updates; contains the robot controller; aggregates the self model | `HeadNode : DeviceNode`, owns `RobotController` and `SelfModel` | `DeviceType::HEAD`, `DeviceTopology::HEAD` |
| Joint | A link with a defined interface and connection rules | `JointNode : DeviceNode` with a `JointInterface` (kind: mechanical, electrical, data bus; allowed child kinds; max children; port types it carries; mount transform) | `DeviceType::LINK`, `DeviceTopology::JOINT`, `AngleJoint` (an actuated joint, kept as an edge actuator) |
| Edge: sensor | Capability → **views** | `Sensor` | `Sensor` (exists) |
| Edge: actuator | Capability → **actions** | `Actuator` | `Actuator` (exists) |
| Edge: communication | Capability → **messages** | `CommunicationDevice` | `CommunicationDevice`, `Emitter`, `Receiver` (exist) |
| Edge: storage | Capability → **capacity** (energy, memory, payload) | `StorageDevice`; `Battery : StorageDevice` | `DeviceType::POWER`, `DeviceType::MEMORY` (enum only) |
| Complex | One physical block, represented as a branch of virtual nodes | `ComplexDevice : DeviceNode`, which builds its branch in `Expand()` | `DeviceType::COMPLEX`, `SpatialObjectSensor`, `FeedbackPair` (both are small composites today) |

`DeviceNode` is the tree node; `Device` (exists) stays the base of everything physical and gains a `DeviceDescriptor`. Every edge node lists `Capability` records:

```
Capability { kind: ACTION | VIEW | MESSAGE | CAPACITY,
             code: ActionType | ViewType | MessageType | ResourceType,
             limits: map<string,double>   // e.g. max_speed, range_m, capacity_Wh, rate_hz
             frame: mount transform from the parent joint }
```

### 5.3 The UAV device tree

The airframe is treated as one complex node, as JB suggested, with a full tree around it as the proof of concept:

```
HeadNode "uav"                        (flight computer: RobotController, SelfModel)
└─ JointNode "frame"                  (mechanical + data bus; accepts: COMPLEX, SENSOR, ACTUATOR, COMM, STORAGE)
   ├─ ComplexDevice "quadrotor"       (one physical block, expanded into a virtual branch)
   │  ├─ Actuator  RotorMotor  "m_fl" (ACTION motor_speed)
   │  ├─ Actuator  RotorMotor  "m_fr"
   │  ├─ Actuator  RotorMotor  "m_rl"
   │  ├─ Actuator  RotorMotor  "m_rr"
   │  ├─ Actuator  FlightControlUnit "fcu"  (combined actuator: ACTION takeoff, land, hold,
   │  │                                      pos_xy, pos_z_yaw, vel_xy, vel_z_yawrate;
   │  │                                      drives m_fl..m_rr through an ActionMap)
   │  ├─ Sensor    Imu          (VIEW attitude, angular_rate, linear_accel)
   │  ├─ Sensor    Gnss         (VIEW geo_position, geo_velocity)
   │  ├─ Sensor    Barometer    (VIEW altitude_baro)
   │  └─ Sensor    Compass      (VIEW heading_mag)
   ├─ Actuator  LedArray "leds"       (ACTION led)
   ├─ StorageDevice Battery "bat"      (CAPACITY energy_Wh, VIEW battery_state)
   ├─ CommunicationDevice Radio "radio" (MESSAGE broadcast, p2p; limits: range_m, bitrate)
   ├─ Sensor RelativePositionSensor "uwb"   (optional; VIEW relative_position_3d of peers)
   └─ Sensor Camera "cam" on JointNode "gimbal"  (optional; VIEW image → detections)
```

Notes:
* **The four motors are real edge actuators** in the tree, as JB asked. The `FlightControlUnit` is a virtual combined actuator. This follows decision 10: a combined actuator takes a **map of actions** matching its sub-actuators, so `fcu` emits an `ActionMap { m_fl: speed, m_fr: speed, m_rl: speed, m_rr: speed }` each tick on Webots.
* **Battery as storage.** The battery's capacity is the energy resource. Its `VIEW battery_state` feeds the worldview, and the resource manager (§7.6) reads its capacity to decide whether a task is affordable.
* **Optional nodes** (relative-position sensor, camera) are simply absent on robots that lack them. Their capabilities then do not appear in the self model, and tasks that require their worldview fields are dumped by that robot (R5).

### 5.4 Building blocks by traversal

`DeviceTree::Traverse(visitor)` walks depth first. `BlockBuilder` is one visitor; it fills:

* `SensorBlock` (exists): all sensors, now also wired to the `ViewRouter`.
* `ActuatorBlock` (exists): all actuators, indexed by **action type → actuator** from their capabilities. This fixes the broken dispatch noted in the assessment and evaluation (`GetActuatorByType` matched actuator type, so drive actions never matched).
* `CommBlock` (new): communication devices, handed to the messenger as transports.
* `StorageBlock` (new): storage devices, handed to the resource manager.

`DeviceBlock` (exists, empty) becomes the owner of the four blocks and of `Init`/`DeInit` for every device, as the master's thesis intended.

Tree updates at runtime (a device fails, a payload is attached) go through the head: `HeadNode::OnTreeChanged()` re-runs the builder and the self model, and raises a capability-change event (§7.3) so the MRS layer can dump tasks the robot can no longer do.

### 5.5 Implement once, reuse: the device registry

`DeviceRegistry` maps a device type name to a factory, so a device class written once can be placed in any robot's tree from a definition file:

```cpp
registry.Register<Gnss>("gnss.webots");          // driver for Webots GPS
registry.Register<GnssMavlink>("gnss.mavlink");  // same capability, ArduPilot driver
registry.Register<Quadrotor>("quadrotor");
```

A robot definition is a text object in the existing protocol style. The master's thesis format rule (identifier, subtype, numeric data, sub-object ids, then sub-objects depth first) is followed; `D` is already reserved for devices in the E-puck `TypeDefinitions::Type`. Illustrative only:

```
D: D_H uav J_1/
J_1: D_J frame mech+bus D_1 D_2 D_3 D_4/
D_1: D_X quadrotor platform=webots/
D_2: D_A leds.webots names=front_left_led,front_right_led/
D_3: D_S battery.webots capacity_Wh=50/
D_4: D_C radio.webots channel=1 range_m=200/
```

`ComplexDevice::Expand()` creates the virtual branch for `quadrotor` from its own descriptor, so the user writes one line for the airframe.

### 5.6 Metadata and the self model

`DeviceDescriptor` (one per node): name, type name, node kind, capabilities, port requirements, mount frame, update rate, mass, power draw, and free-form tags (the existing `Device::tags` map is kept for this).

`SelfModel` is built by the head from all descriptors. It holds:
* the action types the robot can execute and their limits (max speed, max altitude, max climb rate);
* the view types it can produce, and therefore which worldview fields it can fill (§6.7);
* its message capabilities (broadcast, P2P, range);
* its resources (energy capacity, payload);
* the robot type and roles (decision 9: a role is a robot-type requirement).

The self model is what the robot publishes to peers (as its capability profile, §7.7) and what it checks before taking a task.

### 5.7 The generic flight-capable machine

Layers 2 and 3 must not know whether the robot is a quadcopter, a hexacopter or a VTOL (R24). They see only a **generic flight action set** on whichever actuator declares it. Every action keeps a single 64-bit argument, split into two 32-bit floats where two values are needed (decision 10):

| Action | Argument (64 bits) | Meaning |
|---|---|---|
| `TAKEOFF` | float32 target altitude AGL · float32 climb rate | Arm if needed and climb |
| `LAND` | float32 descent rate · unused | Land at current position |
| `HOLD` | float32 duration (0 = until next action) · unused | Hover in place |
| `POS_XY` | float32 x · float32 y (local ENU, m) | Position setpoint, horizontal |
| `POS_Z_YAW` | float32 z (ENU, m) · float32 yaw (rad) | Position setpoint, vertical and heading |
| `VEL_XY` | float32 vx · float32 vy (m/s) | Velocity setpoint, horizontal |
| `VEL_Z_YAWRATE` | float32 vz · float32 yaw rate | Velocity setpoint, vertical and turn |
| `LED` | uint32 pattern · uint32 colour | Existing LED action, kept |
| `WAIT` | duration | Existing delay action, kept (with the inversion bug fixed) |

A full 3D setpoint is an `ActionMap { POS_XY, POS_Z_YAW }` dispatched together to the flight actuator. In the protocol this is a combined action that lists its sub-actions, following the depth-first rule: `A_C 2 A_1 A_2/A_1: A_x 10.0 20.0/A_2: A_z 30.0 1.57/` (illustrative letters).

Drivers behind the same actions:
* **Webots:** `FlightControlUnit` runs the position → velocity → attitude → thrust cascade and mixes to four `RotorMotor` actions.
* **ArduPilot:** `FlightControlUnit` sends `SET_POSITION_TARGET_LOCAL_NED` or `COMMAND_LONG`; the motor nodes exist in the tree for the self model but receive nothing.

---

## 6. The worldview side layer

### 6.1 What stays and what changes

`Worldview` (exists) keeps its three maps (views, parameters, predicates) and `WorldviewType::Spatial3D` (exists in the enum). What changes:

* Storage actually works (`AddView` and `all_views` are empty today).
* Boost timer is replaced by a `Clock` interface (simulation time or wall time).
* Values carry time, source and validity, so "time tracking of view information" (R3) is built in.
* Processing is plug and play (R6), through a router and processors.
* Accessors are typed and fail in a checked way when a field is missing, as the master's thesis requires ("a mismatched worldview must raise an error"). For the allocator this failure is not an error: it is the signal to dump (§6.7).

### 6.2 Objects

| Object | Responsibility |
|---|---|
| `Clock` | `Now()`, `Dt()`; `SimClock` (from the platform) and `WallClock` |
| `WorldField<T>` | One named value: `value`, `stamp`, `source` (sensor, processor or peer id), `valid`, optional variance. Names follow a dotted convention (`pose.enu`, `alt.agl`, `heading`) |
| `TimeSeries<T>` | Fixed-capacity ring buffer of stamped values, with `Latest`, `At(t)` (interpolated), `Window(t0, t1)` |
| `SemanticObject` | Something in the world: `id`, `class` (self, peer, task target, obstacle, detection), 3D pose, velocity, last-seen stamp, source, confidence |
| `ViewRouter` | Receives every view (from sensors and from decoded messages) and forwards it to each processor subscribed to its `ViewType` |
| `IViewProcessor` | `Subscriptions() → set<ViewType>`, `Provides() → set<field name>`, `Process(const View&, Worldview&, t)`, `Tick(Worldview&, t)` for processors that integrate over time |
| `ProcessorChain` | Registration and ordering; a processor that needs another's output declares it, and the chain sorts them once at start |
| `GeoReference` | Fixed WGS-84 origin; converts geodetic ↔ local ENU. Every robot in a team uses the same origin, which arrives in the mission header (§9) |
| `WorldviewRequirements` | A set of field names with a maximum age, e.g. `{pose.enu ≤ 0.5 s, alt.agl ≤ 0.5 s, heading ≤ 0.5 s}`; `Satisfies(const Worldview&, t)` and `Satisfiable(const SelfModel&)` |

The existing parameter and predicate maps become the scalar and boolean cases of `WorldField`. `ParametricAction` reads its inputs by field name, so JB's design ("coefficients are tied to specific Worldview fields") works without change.

### 6.3 One worldview update

Per tick: (1) sensors produce views, (2) the router dispatches them, (3) each processor writes fields and objects, (4) `Tick` runs for estimators, (5) derived predicates are recomputed (`airborne`, `landed`, `battery.low`, `geofence.inside`). Views older than their source's expected period mark their fields stale; a stale required field fails the requirement check exactly like a missing one.

### 6.4 The minimum UAV worldview (R23)

| Field | Type | Processor | Webots input | ArduPilot input |
|---|---|---|---|---|
| `time` | stamp | `Clock` | sim time | sim or wall time, `SYSTEM_TIME` |
| `geo.position` | lat, lon, alt AMSL | `GnssProcessor` | GPS (sim coordinates, converted) | `GLOBAL_POSITION_INT` |
| `pose.enu` | x, y, z | `GeoToLocalProcessor` (uses `GeoReference`) | from `geo.position` | `LOCAL_POSITION_NED` (converted NED → ENU) |
| `alt.agl`, `alt.amsl` | m | `AltitudeProcessor` (baro + GNSS, plus rangefinder if present) | GPS z, barometer if modelled | `VFR_HUD`, `GLOBAL_POSITION_INT.relative_alt` |
| `heading` | rad | `AttitudeProcessor` | inertial unit yaw, compass | `ATTITUDE.yaw` |
| `attitude` | roll, pitch, yaw (quaternion internally) | `AttitudeProcessor` | inertial unit | `ATTITUDE` |
| `vel.enu`, `acc.enu` | m/s, m/s² | `KinematicsEstimator`: alpha-beta (or simple constant-acceleration Kalman) over `pose.enu` history | derived | `LOCAL_POSITION_NED` velocities, then differentiated for acceleration |
| `angular_rate` | rad/s | `AttitudeProcessor` | gyro | `ATTITUDE` rates |
| `battery` | voltage, remaining %, energy Wh | `BatteryProcessor` | battery sensor | `BATTERY_STATUS` |
| `airborne`, `landed`, `armed` | predicates | `FlightStateProcessor` | altitude and speed thresholds | `HEARTBEAT`, `EXTENDED_SYS_STATE` |

Every one of these keeps a `TimeSeries`, so position, velocity and acceleration tracking over time (R3) and the age term of the priority formula (§8.4) come from the same store.

The ArduPilot column shows why processors are plug and play: on hardware the platform already fuses its sensors, so the processor is a pass-through with a frame conversion; in Webots the processors do the work. The fields the tasks read are identical.

### 6.5 Optional worldview content (R4, R23)

| Content | Processor | Notes |
|---|---|---|
| Peer states | `PeerStateProcessor` | Peer heartbeats (§8.6) arrive as `VIEW_PEER_STATE` views and become `SemanticObject{class: peer}`. Needed for separation and for the allocator's peer table |
| Relative positions of drones | `RelativePositionProcessor` | From a UWB-style sensor if fitted. Fuses with peer states, or replaces GNSS-derived peer positions where GNSS is poor |
| Object detections | `DetectionProcessor` | Camera views → detections → `SemanticObject{class: detection}`. Kept optional because on-board compute may not allow vision; the interface lets a detector run off-board or be simulated by the supervisor |
| Coverage or occupancy map | `MapProcessor` (2.5D grid) | Needed only for area-coverage tasks and for obstacle-aware FlyTo; later work |
| Task-site map | `TaskSiteProcessor` | Known task locations and their state from the task pool, for visualisation and for STA |

### 6.6 Worldview synchronisation with peers

Synchronisation is a processor, not a special case. `WorldviewSyncProcessor` publishes selected fields (configurable list, rate and range) through the messenger and turns received ones into views. Fields from peers carry the peer as `source`, so a robot never confuses its own estimate with a peer's. The thesis's `ExchangeInfo` dialog (ask what the peer lacks, send it, merge) becomes the P2P variant of the same processor (§8.6).

### 6.7 Worldview requirements and "insufficient robots dump"

Every task can carry `TaskRequirements` (§7.7), whose worldview part is a `WorldviewRequirements`. Two checks use it:

* **Static** (can this robot ever do it?): `Satisfiable(SelfModel)`. A robot whose sensors cannot produce a required field fails this. Example: a task needing `detections.person` on a robot with no camera.
* **Dynamic** (can it do it now?): `Satisfies(Worldview, t)`. A robot whose GNSS has dropped out fails this temporarily.

The allocator treats a static failure as "dump and never reconsider" and a dynamic failure as "skip for now, reconsider later". Both are broadcast so that peers know (§8.3). This is the general rule JB asked for: algorithms expect that not every robot has the information needed for every task.

---

## 7. Layer 2: the robot layer

### 7.1 The robot controller

`RobotController` lives in the head node and owns the agent loop. It replaces the hand-written loop in `EPuck_Robot.cpp` and finishes `Robot` (exists: `Init`, `Update`, `Exit`, `execute`, `DoAction`, behaviour library). Its responsibilities, as JB listed them:

| Responsibility | Object |
|---|---|
| Initialise the device tree | `HeadNode::Build()` → `PortManager::Assign()` → `DeviceBlock::Init()` → `SelfModel::Build()` |
| Run the agent loop | `RobotController::Tick(dt)`, non-blocking; the platform owns the outer loop |
| Update the world model | `Worldview::Update(t)` through the router and processors |
| Parse and send messages | `Messenger` (§8.6) |
| Allocate its resources | `ResourceManager` (§7.6) |
| Assign actuator actions | `TaskExecutor` → `ActuatorBlock::DoAction` |
| Learning (optional) | `ILearner` hook that receives logged (features, outcome) pairs, for example to train `size_est` (§8.4) |

The master's thesis lifecycle (init, start, action execution, robot update, end, deinit, exit) maps to virtual hooks around `Tick`: `OnInit`, `OnStart`, `OnPreUpdate`, `OnAct`, `OnPostUpdate`, `OnStop`, `OnDeInit`. The existing `Layer` class in `MRS_Algorithms_cpp` already has this hook pattern, so `RobotController` and `MRSLayer` both derive from a Boost-free version of it.

### 7.2 One agent-loop tick

```
Tick(dt):
  ports.Poll()
  sensors.Update(dt)                 → views → ViewRouter → Worldview
  messenger.Receive()                → views (peer state) / MRS messages / events
  worldview.Tick(t)                  → estimators, predicates
  events.Handle()                    → may preempt (§7.3)
  mrs.Step(worldview, t)             → allocator decides the task to pursue
  executor.Step(worldview, t)        → Action / ActionMap
  safety.Filter(action)              → clamp to geofence, separation, limits
  actuators.DoAction(action)         → ports.Write
  messenger.Flush(mrs.Outbox())
  resources.Update(dt); logger.Record()
```

### 7.3 Robustness: events and interrupts (R16)

`EventQueue` holds typed events with a priority. Each tick drains it before normal task work.

| Event | Source | Default handling |
|---|---|---|
| `FAULT` (device lost, port closed, estimator diverged) | device layer, processors | Re-run the self model; if flight-critical, preempt with `EmergencyLand`; otherwise dump tasks that now fail their requirements |
| `BATTERY_LOW`, `BATTERY_CRITICAL` | `BatteryProcessor` | Low: stop taking new tasks, finish the current one only if the energy estimate allows, then `ReturnHome`. Critical: `Land` now |
| `LINK_LOST` | messenger (no peer or operator heartbeat for T) | Keep working on the current task (MRS-RTA does not need comms); after a longer timeout, `ReturnHome` |
| `URGENT_MESSAGE` (operator abort, recall, high-priority task) | messenger | Abort: `Hold` then `Land`. Recall: `ReturnHome`. High-priority task: hand to the allocator immediately, which may preempt |
| `GEOFENCE`, `SEPARATION` | `SafetySupervisor` | Filter or override the setpoint |
| `CAPABILITY_CHANGED` | head node | Re-check every task in the pool against the new self model |

Preemption uses the executor's active-task stack: the interrupting behaviour is pushed on top, and the interrupted task resumes when it ends, if it is still valid. MRS-STA needs the same stack later, so it is built once here.

`SafetySupervisor` is a filter on every outgoing action plus a set of high-priority behaviours. It enforces the geofence, maximum altitude, minimum peer separation (from peer states in the worldview), and battery reserve. These are library features, so no task author has to remember them.

### 7.4 The task executor

`TaskExecutor` implements the master's thesis contract: a task starts when its start condition holds; each step takes the task's next action; the task ends on the **null action**; success or failure is decided by its end condition and its runtime conditions.

Steps, per tick:
1. If the active task's start condition is unmet, look up a behaviour for that condition in the behaviour library (§7.5), bind the condition's values, and push it. Depth is capped (default 2) so behaviours cannot recurse indefinitely.
2. Check runtime conditions (the paper's `C_runtime`, for example "battery above reserve", "inside geofence"); failure ends the task as `FAILED`.
3. Call `Recalculate()` on parametric actions, then `GetNextCommand()` (exists).
4. If the command is the null action, check the end condition and pop.
5. Return the action or action map.

Fixes carried in from the evaluation: wait finishes when elapsed time is **at least** the parameter; the end condition (not the start condition) decides completion; the parser's missing `break` after `CONDITION_NULL`; angle comparison with wrap-around.

### 7.5 The behaviour library and parametric tasks

Decision 4 makes a behaviour a task type, with conditions mapped to behaviour-library tasks. The pieces:

* `BehaviourLibrary` (exists, keyed by name) gains `Find(const Condition&) → Behaviour*`, keyed by condition type with an optional predicate for finer selection, and `Populate`/`PopulateFromFile`, which are empty today.
* `ParameterBinding`: when a behaviour is chosen for a condition, the condition's values are written into the worldview under `target.*` (`target.enu`, `target.yaw`, `target.tol`). The behaviour's parametric actions read those fields. This answers point 2 of the BT analysis §8.5.
* `ParametricAction` (exists) keeps its form (function, worldview, parameter names, scale) and gains:
  * one function per packed half of the 64-bit argument, or one parametric action per entry of an action map;
  * the string form of decision 5: a parsable function string with coefficient count and list (`F_L 2 k1 k2 c field1 field2`), or a registry key (`F_X fly_to_vel`);
  * a fix to `UpdateParameters`, which currently copies parameters by value and never updates them.
* `FunctionRegistry`: `map<string, std::function<double(const Worldview&, Time)>>`, replacing the template-per-class `ExternalFunction<T>`. The library registers the UAV functions (`dist_to_target`, `bearing_to_target`, `vel_to_target_x`, …) so behaviour strings can reference them by name.

**UAV behaviour library.** Shipped as `behaviours/uav.mrsb` in the text protocol, so a user can edit or add behaviours without recompiling.

| Behaviour | Fulfils condition | How it works |
|---|---|---|
| `Liftoff` | `AIRBORNE` predicate, or `ALTITUDE ≥ h` | `TAKEOFF(h, rate)`, end when `alt.agl ≥ h − tol` |
| `Land` | `LANDED` predicate | `LAND(rate)`, end on `landed` |
| `Hover` | `TIME` (wait in the air) | `HOLD(duration)` |
| `ChangeAltitude` | `ALTITUDE = h` | `POS_Z_YAW(h, heading)` repeated until within tolerance |
| `FlyTo.Direct` | `POSITION_3D` | One parametric combined action `{POS_XY(target.x, target.y), POS_Z_YAW(target.z, bearing)}`, repeated until the position condition holds |
| `FlyTo.AltitudeFirst` | `POSITION_3D` | Sequence: climb to cruise altitude, fly horizontally, descend to target altitude. Safer near obstacles and the ground |
| `FlyTo.Layered` | `POSITION_3D` | Like AltitudeFirst, but the cruise altitude is the robot's assigned layer (`base + id × spacing`). This is the default when several UAVs share airspace, because it removes most crossing conflicts and the 2D deadlock problem |
| `FlyTo.Velocity` | `POSITION_3D` | Closed loop on velocity setpoints from registry functions (gain × error, saturated), for platforms or situations where position setpoints are not available |
| `FlyTo.Waypoints` | `PATH` (new condition: path completed) | Sequential task over a list of `POSITION_3D` targets |
| `FlashLED` | none (an action task) | The existing `A_L` LED actions, as in the E-puck task sets |
| `ReturnHome` | `HOME` (position = home, then landed) | `FlyTo.Layered(home)` then `Land` |
| `EmergencyLand` | used by the safety supervisor only | `HOLD` briefly, then `LAND` |

**Conditions added for 3D.** `PositionCondition3D` (ENU target, horizontal and vertical tolerance, optional heading and tolerance, wrap-around for angles), `GeoPositionCondition` (lat, lon, alt; converts through `GeoReference`), `AltitudeCondition`, `PathCondition`, and predicate conditions for `airborne`, `landed`, `home`. These extend the condition family that exists (`ViewCondition`, `ParameterCondition`, `PredicateCondition`, `LogicCondition`, `AbsoluteTimeCondition`) and port the E-puck `ConditionPosition` into the core.

**Example spatial task.** The 2021 E-puck tasks were "go to (x, y), flash LED, wait, flash LED, wait". The UAV equivalent, in the new grammar (illustrative):

```
T: T_A 17 1.0 0 C_1 C_2 R_1 A_1..4/
C_1: C_P3 120.0 -40.0 15.0 1.0 0.5/        start: be at (120, -40, 15) ± 1 m horizontal, ± 0.5 m vertical
C_2: C_N/                                    end: null
R_1: R_F pose.enu alt.agl heading/           requirements: worldview fields
A_1: A_L 17/ A_2: A_W 2.0/ A_3: A_L 0/ A_4: A_N/
```

The robot taking it has no start position, so the executor calls the behaviour mapped to `C_P3`, which is `FlyTo.Layered`, then runs the four actions.

### 7.6 Resource management

`ResourceManager` reads the storage block and tracks:
* **Energy.** Remaining energy from the battery view. An `IEnergyModel` estimates the cost of a task (fly distance at cruise speed plus hover time, using a per-airframe power model whose coefficients come from the device descriptor). A task is feasible only if `cost(here → task) + cost(task) + cost(task → home) + reserve ≤ remaining`. The default model is simple and calibrated in Webots; it can be replaced.
* **Actuator locks.** Two actions that use different actuators can run in parallel (LED while flying); two that use the same one cannot. This is what lets a parallel task run LED and flight branches on one robot.
* **Communication budget** (optional): limits message rate when the transport reports a low bitrate. The paper notes that CBAA's results were shaped by a 152 baud link, so the comparison needs this to be explicit.

The resource manager feeds both the dump decision (§8.3) and the priority estimate (§8.4).

### 7.7 Task requirements and the capability profile

`TaskRequirements` is an optional part of every task:

| Part | Example | Checked against |
|---|---|---|
| Worldview fields | `pose.enu`, `alt.agl`, `detections.person` | `WorldviewRequirements` (§6.7) |
| Action capabilities | `LED`, `POS_XY` | `SelfModel` action list |
| Role / robot type | `role=camera_uav` | `SelfModel` roles (decision 9) |
| Resources | `energy_Wh ≥ 5`, `payload_kg ≥ 0.2` | `ResourceManager` |
| Affinity | `same_robot_as=T_12` (interdependent task, master's thesis) | allocator |

The robot publishes a `CapabilityProfile` (a compact digest of the self model and current resources) to peers when it joins and when it changes. Allocators that coordinate (CBBA, STA) use peers' profiles to avoid offering a task to a robot that cannot do it. This is also the D1 capability set from the master's thesis, so the same data serves SCA later.

---

## 8. Layer 3: the MRS layer

### 8.1 Objects

| Object | Responsibility | From |
|---|---|---|
| `MRSLayer` | Hosts the allocator, the task pool and the peer table; runs one step per tick; owns the outbox | `Layer` (exists) without Boost |
| `TaskPool` | All tasks this robot knows: global id (issuer + sequence), task object, state (`NEW`, `AVAILABLE`, `CLAIMED_BY(r)`, `ACTIVE`, `DONE`, `FAILED`, `DUMPED`, `IMPOSSIBLE`), arrival time, history | Replaces E-puck `TaskPriorityList` and fixes its overwrite bug by keying on ids instead of a moving size counter |
| `PeerTable` | The list of identified agents (master's thesis): id, last state, capability profile, current task or claim, last heard | Fills the role of the never-filled `RobotInfo` array |
| `IAllocator` | The generic seam (below) | Decision 1 |
| `ISizeEstimator` | `size_est(task, worldview, t)`, hardcoded or learned | Decision 2 |
| `ITaskDecomposer` | Splits a tree task into base tasks plus constraints (§8.7) | New |

### 8.2 The allocator interface

The interface drafted in the earlier analyses, made concrete:

```cpp
class IAllocator {
public:
  virtual void Bind(TaskPool&, PeerTable&, const SelfModel&, ISizeEstimator&, Outbox&) = 0;
  virtual void OnTaskReceived(const TaskId&, Time t) = 0;
  virtual void OnMessage(const Comm::Message&, Time t) = 0;
  virtual void OnTaskFinished(const TaskId&, TaskResult, Time t) = 0;   // done, failed, dumped
  virtual void OnCapabilityChanged(Time t) = 0;
  virtual Decision Select(const Worldview&, Time t) = 0;   // keep, switch(TaskId), idle
  virtual AllocatorInfo Info() const = 0;                  // name, exclusive?, uses comms?
};
```

The same seam covers behaviour-based self-selection (RTA, STA, LDTA²), market-based allocation (CBBA) and the planner baseline. Allocators never touch devices; they read the worldview and the pool, and write messages to the outbox. That makes each one testable without a simulator.

### 8.3 Dumping (R5)

Dump is a library rule, used by all allocators, so it is never reimplemented:

1. A task fails the **static** check (§6.7, §7.7) → state `DUMPED_SELF` forever on this robot; broadcast `DUMP(task, reason)`.
2. A task fails the **dynamic** check or the energy check → skip it this tick; reconsider when the worldview or resources change.
3. A task the robot started but cannot finish (runtime condition fails, fault) → `DUMP` broadcast with progress, task returns to `AVAILABLE` for others.
4. A task every known peer has dumped → `IMPOSSIBLE`, reported to the operator.

This settles open item 2 of the total analysis for the UAV case.

### 8.4 The priority estimate

`Priority = base_priority × size_est(task, w, t)` (decision 2), with base priority applied **once** (the E-puck code applies it twice).

Default `SpatialSizeEstimator` for UAVs extends the paper's formula (5), `P = b(a1·Δt + a2·Δs⁻¹)`, to 3D and energy:

```
size_est = a1·Δt + a2 / (Δs₃ + ε) − a3·energy_fraction(task) − a4·claim_penalty
   Δt   = time since the task was issued (from the TimeSeries / task stamp)
   Δs₃  = 3D travel distance with the chosen FlyTo variant (layered: up + across + down)
   energy_fraction = estimated task energy / remaining energy (0 if unknown)
   claim_penalty   = 1 if a peer closer to the task has claimed it (exclusive mode)
```

Coefficients are configuration, not code. A learned estimator implements the same interface; the `ILearner` hook logs features and outcomes so it can be trained later.

### 8.5 Allocators, in order

| Order | Allocator | Why here | What it needs from the library |
|---|---|---|---|
| 1 | **MRS-RTA** | First by decision; behaviour-based, comms-light; the paper's algorithm | Task pool, `size_est`, behaviour fulfilment, completion broadcast. New over 2021: optional **exclusive mode** (claims), dump rule, energy feasibility, timestamps actually set |
| 2 | **Planner baseline** | Needed for the evaluation question (§10). Not an MRS algorithm; a reference | A single-robot route planner over all known tasks: nearest neighbour + 2-opt on 3D travel time, replanning when a task arrives. The same class with N robots and a central planner (greedy insertion mTSP) gives an "ideal coordination" reference |
| 3 | **CBBA** | The paper's own "state of the art" reference, and it is in JB's scope. Fixes the CBAA shortcuts (stepping the simulator inside the allocator, hard-coded team mask) by design | Broadcast bids, P2P replies, peer table, bundle and path per robot |
| 4 | **MRS-STA** | Needed for tree tasks split between robots (R22) | Active-task stack (§7.3), `ITaskDecomposer`, subtask states shared through messages (`peer_active`, `peer_done`), affinity |
| 5 | **LDTA²** | Swarm-proportion comparison in JB's scope | Peer counts per task type from the peer table |
| 6 | **MRS-TTA** | Team task allocation; mechanism still undefined | Placeholder slot; likely builds on STA's subtask sharing and multi-robot tasks |

MRS-SCA stays out of this plan, as decided; its `IBehaviourSelector` seam can later read the same behaviour library and worldview.

### 8.6 Communication: broadcast and P2P

`MRS::Comm::Message` is the one envelope: `sender`, optional `recipient` (none = broadcast), `type`, `task id`, `stamp`, `payload` (text protocol). `ITransport` offers `Broadcast`, `Send(peer)`, `Poll`, and reports its limits (range, bitrate). A transport without one mode emulates it, as the BT analysis §8.3 describes.

Message types:

| Type | Mode | Used by |
|---|---|---|
| `TASK` | broadcast from the issuer (or relayed) | all |
| `CLAIM`, `RELEASE` | broadcast | RTA exclusive mode, STA |
| `DONE`, `FAILED`, `DUMP` | broadcast | all |
| `STATE` (heartbeat: pose, velocity, battery, current task) | broadcast at a fixed rate | peer table, separation, worldview sync |
| `PROFILE` | broadcast on join or change | capability checks |
| `BID`, `WINNERS` | broadcast | CBBA |
| `INFO_REQ`, `INFO` | P2P | `ExchangeInfo` gossip, worldview sync on demand |
| `CMD` (abort, recall, pause) | broadcast or P2P from the operator | urgent-message event |

`Messenger` decodes, routes (`STATE` → worldview as a view; allocation messages → `MRSLayer`; `CMD` → event queue) and enforces the communication budget.

### 8.7 Tree tasks split between robots (end goal of R22)

The task tree uses the master's thesis links: **sequential, parallel, choice, defined**, plus affinity (interdependent tasks).

1. **Decompose.** `ITaskDecomposer` turns a `ComplexTask` (fixed so it compiles) into its leaf tasks plus a constraint set: precedence edges from sequential links, "any k of n" from parallel links, exclusive alternatives from choice links, and same-robot affinity.
2. **Publish.** Leaves become ordinary pool entries with a `parent` id and constraints. A leaf whose predecessors are not done is `BLOCKED` (expressed in the protocol as a `CONDITION_SUBTASK` start condition, which exists in the condition enum).
3. **Allocate.** Any allocator can take unblocked leaves; RTA treats them like any task, STA additionally keeps the parent on its active-task stack and prefers leaves of trees it is already working on. Affinity forces a leaf to the robot that did its partner.
4. **Report.** Leaf `DONE` messages unblock successors; the parent is `DONE` when its link rule is satisfied.

Worked UAV example: "inspect the area" = parallel(coverage strip 1 … strip k) → sequential → "all land at home". Strips are split across robots; landing waits for all strips.

---

## 9. Operator side: task issuer and task generation (R18)

### 9.1 Task issuer

`TaskIssuer` is the operator's agent. In simulation it is a Webots supervisor, like `operator_robot` today. On hardware it is a ground-station process using the same code. It:
* sends the **mission header** (geo origin, geofence, home positions, altitude layers, team list) to every robot;
* dispatches tasks from a timeline (static: all at t = 0; dynamic: at given times), keeping the 2021 `.dat` timeline idea (`time  T: …`) and fixing its sorting bug;
* receives `DONE`/`FAILED`/`IMPOSSIBLE` and logs per-task timing;
* can issue `CMD` messages (abort, recall).

### 9.2 Task generators

`ITaskGenerator::Generate(OperatorInput, GeoReference, Params) → vector<Task string>` converts operator-domain input to protocol strings. Input arrives as GeoJSON (points, line strings, polygons) or simple CSV of lat/lon/alt, because both are easy to produce from GIS tools.

| Generator | Input | Output |
|---|---|---|
| `PointTaskGenerator` | Points with altitude and an action template (e.g. "flash LED, hover 2 s") | One atomic spatial task per point |
| `PathTaskGenerator` | Line string | A sequential task of waypoint tasks, or one `FlyTo.Waypoints` task |
| `AreaCoverageGenerator` | Polygon, sensor footprint, overlap, altitude | Boustrophedon (lawnmower) strips cut into k cells; a parallel tree task of cell tasks (§8.7) |
| `PerimeterGenerator` | Polygon | Sequential segments along the boundary, splittable into arcs |
| `RandomSpatialGenerator` | Bounds, count, distributions | The evaluation task sets: uniform, clustered, multi-cluster, radial, with deterministic or random dispatch, mirroring the 2021 families (§10.3) |

Generators live in the library; the operator supplies only the input file and parameters (R26).

---

## 10. Evaluation: group vs a single planned UAV (R25)

### 10.1 Question and hypothesis

**Question.** For a set of spatial tasks, is the makespan of N UAVs (MRS-RTA, later CBBA and STA) shorter than that of one UAV flying a planned mission over the same set?

**Expected result, to be tested.** Yes for most sets, with a speed-up below N, because of takeoff and landing overhead, travel to spread-out clusters, and duplicate travel without claims. The gap should be largest for spread-out sets and dynamic dispatch, and smallest for a single tight cluster. Battery limits may make some sets impossible for one UAV without recharging, which needs a rule (question 2 in §14).

### 10.2 Conditions

| Condition | Robots | Allocation | Knows the future? |
|---|---|---|---|
| S1: single, planned | 1 | Planner baseline, replans on each arrival | No (dynamic) / yes (static) |
| S1*: single, oracle | 1 | Planner with the full timeline in advance | Yes (lower bound for one UAV) |
| G-RTA | N ∈ {2, 3, 5, 8} | MRS-RTA, open mode | No |
| G-RTA-X | N | MRS-RTA, exclusive mode (claims) | No |
| G-CBBA | N | CBBA | No |
| G-C | N | Central mTSP planner | Static: yes; dynamic: replans |
| G-STA | N | MRS-STA (tree-task sets only) | No |

Same Webots world, same airframe, same FlyTo variant (`Layered` for groups, `AltitudeFirst` for the single UAV, so the single UAV is not penalised by layers it does not need), same task sets and seeds.

### 10.3 Task sets

* **Ported 2D sets.** The 25 E-puck task sets, scaled to the flying area and given an altitude (fixed, or drawn from a range), so 2D and 3D results can be compared.
* **New 3D sets.** Same families (even grid, radial, single and multi-cluster, random), with altitude variation, and with static and dynamic dispatch (even, clustered, random timing).
* **Tree-task sets** (for WP8): area coverage and perimeter missions from the generators.
* **Scale:** 15, 30 and 60 tasks; at least 10 seeds per set and condition (the 2021 results were single runs, which the evaluation noted as the main weakness).

### 10.4 Metrics

Primary: **makespan** (first dispatch to last completion). Secondary: mean and maximum task latency (dispatch to completion), total distance flown, total energy used, duplicate arrivals (robots per task), messages and bytes sent, decision time per tick (mean and worst), safety events (separation breaches, geofence hits), and failures or stalls. Every run writes metadata next to its results: commit, world file, task set, seed, N, allocator and coefficients.

Analysis: paired comparison per task set and seed (group vs S1), speed-up = makespan(S1) / makespan(group), with confidence intervals across seeds, broken down by spatial family and dispatch type. A small Python script in `experiments/uav_spatial` produces the tables and plots.

### 10.5 Harness

`experiments/uav_spatial/` holds the Webots worlds (one per N, generated from a template), the supervisor issuer, a batch runner that launches Webots in fast mode without rendering for each (set, seed, condition), and the analysis script. The batch runner is the only piece that is Windows/Linux specific.

---

## 11. Work packages

Each package ends with something that runs and an acceptance check. The packages are in dependency order; packages on separate lines in the same group can run in parallel.

| WP | Content | Acceptance |
|---|---|---|
| **WP0 Spec and skeleton** | Grammar (EBNF) for tasks, conditions, actions incl. combined actions and 64-bit packing, devices, requirements, messages; CMake + C++17 skeleton, no Boost; CI on Windows and Linux; unit-test framework | Round-trip parse/write tests for every object kind; skeleton builds on both systems |
| **WP1 Core repair** | Port the parts of the core the UAV needs: `Action` (one 64-bit argument), `ActionMap`, `Task`, `ATask`, `Behaviour`, condition family incl. `PositionCondition3D`, `ParametricAction` + `FunctionRegistry`, `BehaviourLibrary::Find/Populate`, `ComplexTask` family compiling | Unit tests: task strings run step by step against a scripted worldview; null-action rule; wait ≥ t; parametric values update every tick |
| **WP2 Port layer + device tree** | `Port`, `PortManager`, `IPlatform`; `DeviceNode`, `HeadNode`, `JointNode`, `ComplexDevice`, `StorageDevice`, `Capability`, `DeviceRegistry`, `BlockBuilder`, `SelfModel`; `WebotsPlatform`; UAV device classes; robot definition file | A Webots Mavic 2 Pro is built entirely from a definition file; the self model lists the expected actions, views, messages and capacity; removing the GNSS line removes `pose.enu` from what the robot can provide |
| **WP3 Worldview pipeline** | `Clock`, `WorldField`, `TimeSeries`, `SemanticObject`, `ViewRouter`, `IViewProcessor`, `GeoReference`, `WorldviewRequirements`; processors for the minimum fields (§6.4) | In Webots, the logged `pose.enu`, `alt.agl`, `heading`, `vel.enu` match supervisor ground truth within set tolerances; a processor can be added or removed without touching others |
| **WP4 Single-UAV flight** | `FlightControlUnit` for Webots (cascaded control + mixer), `RobotController` with the agent loop, `TaskExecutor`, `SafetySupervisor`, `ResourceManager` + energy model, UAV behaviour library file | One UAV executes Liftoff → FlyTo (each variant) → FlashLED → Land from task strings; geofence and low-battery behaviours trigger correctly; energy estimate within a set error of the simulated battery |
| **WP5 Comms + MRS-RTA** | `Message`, `ITransport`, `Messenger`, `WebotsRadioTransport`; `MRSLayer`, `TaskPool`, `PeerTable`, `IAllocator`, `SpatialSizeEstimator`, `RtaAllocator` (open and exclusive), dump rule; `TaskIssuer` supervisor | N = 5 UAVs complete a ported 2021 task set with no stalls; a UAV built without LEDs dumps LED tasks and the others do them |
| **WP6 Baseline + evaluation v1** | `PlannerAllocator` (single and central), task-set generators for §10.3, batch runner, analysis script | **First answer to the core question**: makespan tables for S1, S1*, G-RTA, G-RTA-X, G-C over the ported and new sets, ≥ 10 seeds |
| **WP7 Tree tasks + generators** | `ITaskDecomposer`, leaf constraints and blocking, `PointTaskGenerator`, `PathTaskGenerator`, `AreaCoverageGenerator`, `PerimeterGenerator`, GeoJSON input | A polygon in GeoJSON becomes a parallel coverage tree; RTA completes it across 3 UAVs with correct precedence (landing waits for all strips) |
| **WP8 MRS-STA** | `StaAllocator` with the active-task stack and subtask sharing; affinity | **End goal of the MRS layer**: a tree task is split into base tasks and shared between robots under STA; evaluation v2 adds G-STA on tree sets |
| **WP9 CBBA, LDTA²** | `CbbaAllocator`, `Ldta2Allocator`; communication budget for bitrate-limited runs | Evaluation v3: all allocators on the same sets; CBBA results reported at two bitrates |
| **WP10 ArduPilot platform** | `ArduPilotPlatform`, `MavlinkPort`, MAVLink pass-through processors, `FlightControlUnit` in GUIDED mode, `UdpTransport` | Same task strings and the same allocator run against ArduPilot SITL (alone, then with Webots); then a single real UAV flight test of Liftoff/FlyTo/Land |
| **WP11 Optional sensing** | `RelativePositionProcessor`, `WorldviewSyncProcessor`, `DetectionProcessor`, `MapProcessor`; `ILearner` + learned `size_est` | Separation and allocation use relative positions when available; a learned estimator plugs in without allocator changes |

The C API, Python and C# bindings and the ROS 2 wrapper from the earlier plan follow WP6 or later and are not on the critical path for the evaluation.

---

## 12. What the user defines (R26)

| The user writes | Through | Default the library provides |
|---|---|---|
| Tasks | Text-protocol strings, or a generator input file (GeoJSON/CSV) plus parameters | Generators (§9.2); example task sets |
| Hardware interface | An `IPlatform` implementation (ports, clock, step) | `WebotsPlatform`, `ArduPilotPlatform` |
| Communication | An `ITransport` implementation | `WebotsRadioTransport`, `UdpTransport` |
| Robot definition | A device-tree definition file | Mavic 2 Pro (Webots) and ArduPilot quad definitions |
| Tuning (optional) | Config: control gains, `size_est` coefficients, altitude layers, geofence, battery reserve | Calibrated defaults |
| New device types (optional) | A `Sensor`/`Actuator`/`StorageDevice` subclass registered in `DeviceRegistry` | The UAV device set |
| New behaviours (optional) | Lines in the behaviour library file, or functions in the `FunctionRegistry` | The UAV behaviour library |

Everything else (worldview processing, execution, safety, resource checks, allocation, decomposition, messaging, logging, evaluation) is library code.

---

## 13. Existing objects and what happens to them (R27)

| Existing object | Fate | Change |
|---|---|---|
| `Device` | Extend | Gains `DeviceDescriptor`, capabilities; keeps `tags`, `Init`/`DeInit` |
| `DeviceType` enum | Extend | Keep values; add `STORAGE`, `COMM` as edge kinds (POWER/MEMORY become storage subtypes) |
| `DeviceTree`, `DeviceTopology` | Rewrite internals | Real insertion, iterator, traversal, head/joint/edge/complex nodes |
| `DeviceBlock` | Complete | Owns the four blocks and device lifecycle |
| `Sensor`, `SensorBlock` | Extend | Views go to the `ViewRouter`; `SensorType` widened for UAV sensors |
| `Actuator`, `ActuatorBlock` | Extend + fix | Dispatch by action type via capabilities; `ActionMap` support |
| `CommunicationDevice`, `Emitter`, `Receiver` | Keep | Become the device side of `ITransport` |
| `FeedbackPair`, `AngleJoint` | Keep | Used for gimbals; `FeedbackPair` is a small `ComplexDevice` |
| `Controller` | Generalise | Becomes `Port` / `ProtocolPort` in layer 0 |
| `Action`, `ActionType` | Extend + fix | One 64-bit argument (drop `actionParameter2`), split helpers kept (`FirstHalfAsDouble` etc. exist), generic flight actions added |
| `ParametricAction`, `MathFunction` | Extend + fix | Per-half functions, string form, `FunctionRegistry` replaces `ExternalFunction<T>`, `UpdateParameters` fixed |
| `View`, `ViewType`, `View2DPosition`, `ObjectView`, `InformationView` | Extend | 3D view types implemented (enum entries exist); stamps from `Clock`; new geo, attitude, battery, peer-state views |
| `Worldview`, `WorldviewType` | Extend | Router, processors, fields, time series, requirements |
| `Task`, `TaskType`, `TaskState` | Extend | Global ids, requirements, runtime conditions, `DUMPED`/`IMPOSSIBLE`/`BLOCKED` states |
| `ATask`, `ParametricATask`, `Behaviour`, `BehaviourLibrary` | Complete | As in §7.4–7.5 |
| `ComplexTask`, `SequentialTask`, `ParalelTask`, `OptionalTask` | Fix | Compile; link semantics from §8.7 |
| `GoalTask`, `UtilityTask` | Leave | Not needed for the UAV evaluation |
| Condition family | Extend | 3D, geo, altitude, path, flight predicates; E-puck `ConditionPosition` folded in |
| `Agent`, `Robot`, `RobotInfo` | Extend | `RobotController` and lifecycle; `RobotInfo` becomes `SelfModel` output |
| `Layer`, `SimpleLayer` | Port | Without Boost; base of `RobotController` and `MRSLayer` |
| `MRS_STA` (empty) | Implement | WP8 |
| `MRS_SCA`, `StatePredictor`, `ValueCalculator`, `Value` | Leave | Separate later module, as decided |
| E-puck `TaskPriorityList` | Replace | `TaskPool` |
| E-puck `MessageTranslator`, `TypeDefinitions` message enums | Replace | `MRS::Comm` |
| E-puck `CBAA_Manager` | Replace | `CbbaAllocator` behind `IAllocator` |
| E-puck `MRS_Robot` | Fold in | Its methods map onto `RobotController` and `IAllocator` |
| E-puck `operator_robot` | Replace | `TaskIssuer` supervisor |

The E-puck app itself is not changed by this plan. Once WP5 exists, porting it to the new core gives a 2D regression benchmark for free.

---

## 14. Risks and open questions

### 14.1 Risks

| Risk | Effect | Mitigation |
|---|---|---|
| Webots flight control tuning takes time | Delays WP4 | Start from Webots' Mavic example gains; keep `FlyTo.Velocity` as a fallback; consider ArduPilot SITL in Webots earlier if tuning stalls |
| Simulation speed with 8 drones | Fewer seeds | Fast mode without rendering; batch runner; run N = 8 only on a subset |
| Planner baseline seen as weak | Weakens the conclusion | Include the oracle (S1*) and the central mTSP (G-C) as references |
| GNSS realism in Webots (noise-free by default) | Optimistic results | Add GNSS noise and dropouts as a world setting; record it in run metadata |
| ArduPilot GUIDED-mode behaviour differs from Webots FCU | Different flight times between platforms | Compare SITL and Webots FCU on the same tasks in WP10 and report the difference |
| Text grammar churn | Breaks task sets | Grammar fixed in WP0 with round-trip tests; task sets regenerated by generators, not hand-edited |

### 14.2 Questions for JB

Each has a default that the plan already uses, so none blocks the start.

1. **Single-UAV baseline:** is "mission planning" a route planned over all known tasks and replanned when new tasks arrive (default), or a fixed pre-planned route?
2. **Battery in the evaluation:** when one UAV cannot finish a set on one battery, should it land and swap (fixed swap time, default), or should such sets be excluded?
3. **Simulated airframe:** Webots' Mavic 2 Pro with the library's own flight controller (default), or ArduPilot SITL inside Webots from the start?
4. **Inter-UAV collision handling:** altitude layers plus a minimum-separation filter (default), or full reactive avoidance?
5. **Repository:** the 2021 paper cites `github.com/Redestroy/MRS` (branch *Dynamic-Spatial-Task-allocation-in-Multi-robot-systems*). Should implementation happen in that repository, and can it be attached to this project?

---

## 15. Decisions recorded (JB, 2026-10-04, after this plan)

These answer §14.2 and override the plan wherever they differ.

1. **Single-UAV baseline:** the default holds. The single UAV flies a route planned over all known tasks and replans when a new task arrives (condition S1, §10.2).
2. **Battery swaps:** a UAV must **save its tasks** so that after a battery swap and restart it **resumes after liftoff**. Consequences for the plan:
   * New `TaskJournal` in layer 2. It writes the task pool, the active task with its action iterator, the active-task stack and the claims to a `StorageDevice` (memory) node on every task-state change. The robot's identity and the mission header are written too. It uses the text protocol, so the journal is human-readable.
   * New lifecycle step in `RobotController::OnInit`: if a journal exists for the current mission, reload it, run `Liftoff`, then resume the interrupted task. A fly-to resumes from where the robot is now, because its start condition is checked again; atomic actions already done are skipped by the iterator. Peers are told with a `STATE` message that the robot is back.
   * While the robot is down, its claimed tasks stay claimed for a configurable grace period, then return to `AVAILABLE` (dump rule 3, §8.3). This stops peers from taking the work during a short swap but prevents it from being lost on a long one.
   * In the evaluation, a swap is modelled as land at home, a fixed swap time, a restart from the journal, and liftoff. The swap time counts in the makespan. This replaces the "exclude the task set" option.
   * Placement: `TaskJournal` and the resume step join **WP4** (single-UAV flight). WP4's acceptance adds: "a UAV interrupted by a simulated battery swap resumes its task list after liftoff".
3. **Airframe:** start with the Webots Mavic 2 Pro and the library's own `FlightControlUnit`. ArduPilot follows later (WP10, unchanged).
4. **Collision handling:** altitude layer selection by robot ID (`FlyTo.Layered`) is the starting point. The **long-term goal is full reactive avoidance based on whatever sensor information is available**. Consequences:
   * In the plan, `SafetySupervisor`'s separation filter is the first step. A later work package, **WP12 Reactive avoidance**, adds an `IAvoidance` strategy in layer 2 that reads peer states, relative-position views and detections or maps from the worldview. Velocity obstacles or potential fields are candidates. It uses what the robot actually has, following the same worldview-requirements rule as tasks: with no peer information it falls back to layers only.
   * `IAvoidance` sits between the executor and the actuators (step `safety.Filter` in §7.2), so tasks and allocators do not change when it is added.
5. **Repository:** implementation goes to `github.com/Redestroy/MRS` on a **new branch `spatial-3d`**.
