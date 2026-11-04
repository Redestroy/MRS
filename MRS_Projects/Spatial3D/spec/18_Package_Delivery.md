# 18 Package delivery

Status: **draft**, started 2026-11-04 from JB's delivery demo request. Plan: `/mnt/project-files/delivery/Package_Delivery_Plan.md` (project files). Sections marked *pending* wait for JB's answers to the plan's §5 questions; the defaults there are what this draft assumes.

## 1. Scope

A team of UAVs delivers small packages from a pile at a source point to destination points, carrying each package on a sling with a hook. One delivery is one bound unit (spec 12 §2.1): the robot that hooks a package is the one that releases it. The demo adds:

* a sling actuator and its views (§2, *pending* Q1–Q2);
* behaviours: pick up, drop off, wait in line, package flight, normal flight, recharge (§3, *pending*);
* shared zones with queues (§4, *pending*);
* a delivery generator and a package manifest (§5, *pending* Q3);
* a range-limited radio and message relaying (§6);
* intermediate drop points for deliveries out of loaded range (§7, *pending* Q4).

## 2. Sling (*pending*)

## 3. Behaviours (*pending*)

## 4. Zones and queues (*pending*)

## 5. Generator and manifest (*pending*)

## 6. Radio range and relaying

### 6.1 Range in the test simulator

`Sim::Air::range_m` (`TeamConfig::radio_range`, m; default 0 = everyone hears everyone). Above 0, a message reaches only the agents within `range_m` (3D distance) of its sender at the moment it goes on the air. The operator's radio stands at `Air::operator_pos` (`TeamConfig::operator_pos`, ENU). Each reception lost to range is counted in `Air::out_of_range`. Range applies after the bitrate queue (spec 13 §3): a queued message is checked against the positions when it is sent, not when it was queued.

### 6.2 Relaying

`MrsConfig::relay` (default off) turns on `Messenger::SetRelay`. A relaying agent sends on, once and unchanged, every message it hears that:

* parses and has a known MAJOR version;
* is not its own;
* is not a unicast to itself;

whether or not it is for this agent's mission, and whether or not the agent has seen it before through its own receive path. Each `(sender, seq)` pair is sent on at most once per agent, so a flood over any graph ends after every agent has sent each message once. A relayed message keeps its envelope (sender, recipient, seq, stamp), so the receivers' duplicate rule (spec 06 §2) drops second copies and the recipient of a unicast sees it as sent by the original sender. `MessengerStats::relayed` counts the messages sent on.

Every robot relays when the option is on; there are no designated relay robots in the message layer. Consequences:

* the channel load grows with the number of relaying agents (each message is sent up to once per agent). On a limited bitrate (spec 13 §3) this is the cost of the chain;
* the spec 15 rule that sync is "never relayed" is about the content (a robot does not re-publish fields it learned from peers in its own `M_SYNC`); relaying the original `M_SYNC` message unchanged is allowed;
* the issuer does not relay.

### 6.3 Relay positions (*pending* Q5)

When a delivery's destination is beyond the issuer's range and no chain of robots covers the gap, the generator adds hold-position tasks at points along the line, spaced below `range_m`, for idle robots. Details follow the answer to Q5.

## 7. Intermediate drop points (*pending*)

## 8. Tests

* Relay (`tests/delivery/test_relay.cpp`): on a three-agent chain, a broadcast crosses once per agent; hearing it again sends nothing; a unicast passes through a relay that does not take it; a unicast to the relay is not sent on.
* Range: three QuadSim robots 25 m apart, the operator 20 m before the first, range 30 m. Without relay only the first robot learns the mission within 5 s; with relay all three do, and the first two have relayed messages.
