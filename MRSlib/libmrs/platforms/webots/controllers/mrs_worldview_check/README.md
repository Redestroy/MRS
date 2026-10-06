# mrs_worldview_check

WP3 acceptance in Webots: the worldview's `pose.enu`, `alt.agl`, `heading` and `vel.enu` against supervisor ground truth.

1. Build libmrs with `WEBOTS_HOME` set. The controller binary and `mavic_webots.mrsd` land in this folder.
2. Copy the Cyberbotics world `projects/robots/dji/mavic/worlds/mavic_2_pro.wbt` and change it:
   * `WorldInfo`: `gpsCoordinateSystem "WGS84"`, `gpsReference 56.9496 24.1052 10` (any reference works).
   * The `Mavic2Pro` node: `controller "mrs_worldview_check"`, `supervisor TRUE`.
   * Optional: an `Emitter` named `"emitter"` and a `Receiver` named `"receiver"` in `bodySlot`. Without them the radio is reported as a FAULT and the check still runs.
3. Point the world at this controller folder (copy the folder into the world's `controllers/`).
4. Run. The drone climbs to 5 m, then flies forward while turning. After 40 s the console prints the maximum errors and PASS or FAIL. Tolerances: pose 0.5 m, altitude 0.5 m, heading 0.05 rad, velocity 0.3 m/s. The full log is `mrs_worldview_check.csv`.

The flight uses the stabiliser of the Cyberbotics Mavic example, only to make the fields change; position control is WP4.
