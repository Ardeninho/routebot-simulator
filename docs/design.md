# Design notes

## Planner versus controller

The planner chooses **visit order**; the controller generates **motion commands**. A shorter tour alone cannot drive a vehicle. Separating these responsibilities allows the planner to change without changing device drivers, and a hardware adapter to change without rewriting route selection.

Coordinates are bounded to ±1000 meters on each axis. Missions contain 2–32 stops. Stop 0 is the depot, and the simulated robot starts there at heading zero. The library caller must provide a pose corresponding to its own mission start. The core owns copies of the mission and route, so the caller need not keep them alive after starting the controller. Controller start validates route indices and uniqueness before accepting a mission.

Nearest insertion begins with the closest pair, caches each unused station's distance to the current tour, inserts the closest station at the least-cost edge, and updates the cache only against that station. This yields O(n²) time with O(n) storage. All algorithms normalize the resulting cycle to depot 0 and the specified index-based orientation.

## Controller contract

The host invokes `controller_tick` once per 20 simulated milliseconds. HAL callbacks are synchronous and must return promptly; the API expects valid controller and callback pointers. Wheel velocities use meters per second; headings use radians. No application heap allocation occurs in the core. Host stdio is outside that core.

For distance d and wrapped heading error e, forward speed is `min(0.6, d) * max(0, cos(e))`. Forward speed is zero for |e| > 0.7 radians, allowing the robot to turn first. Angular speed is `clamp(3e, -2, 2)` rad/s. With track width b = 0.40 m, left/right wheel speeds are `v ∓ omega*b/2`, bounded to ±0.8 m/s. A waypoint is accepted within 0.06 m.

This is a proportional geometric controller with an ideal wheel-velocity actuator. It is not a PID wheel-speed controller and has no encoder feedback loop. The plant integrates wheel speeds using a midpoint heading approximation.

## States and faults

| Condition | Result |
| --- | --- |
| Valid mission start | RUNNING |
| Obstacle asserted | BLOCKED; zero wheel command |
| Obstacle clears | RUNNING; continue same target |
| Failed sensor read, nonfinite pose or sample older than 100 ms | Latched SENSOR_FAULT; zero command |
| Emergency stop asserted | Latched ESTOP_FAULT; zero command |
| Mission elapsed time exceeds 300 s, including blocked time | Latched TIMEOUT_FAULT; zero command |
| Last waypoint reached | COMPLETE; zero command |
| FAULT or COMPLETE on later ticks | Remain stopped |

Fault recovery requires an explicit new `controller_start`; merely clearing a signal cannot restart motion. Every tick writes a command, including zero in inactive states. Independently, the simulated motor device stops when the last command is over 100 ms old. This models a device watchdog but is not a physical emergency-stop circuit. Unsigned elapsed-time arithmetic handles a timer wrap when intervals remain below one full 32-bit timer period.

Sensor validity is checked before emergency stop, so simultaneous sensor failure and stop assertion reports SENSOR_FAULT; both command zero motion. Stopping is instantaneous in this idealized plant. Physical braking distance, inertia, communication loss and electrical failure are not modeled.

## Next hardware boundary

A future adapter could implement `Hal.read` from localization/encoders and `Hal.write` through a motor driver's velocity interface. That would also require calibration, scheduling, motor control, independent stop wiring and actual hardware validation. A bare PWM command is not interchangeable with the current velocity command. Start by testing one motor's measured speed before attempting autonomous navigation.
