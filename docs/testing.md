# Verification

`make test` runs C checks and four complete simulator scenarios. `make sanitize` rebuilds both executables with AddressSanitizer and UndefinedBehaviorSanitizer and runs the same suite. These commands were run successfully on the development Mac. GitHub Actions is configured but has not yet run on a remote repository.

C checks cover square-route length, route permutations, maximum capacity, invalid/nonfinite coordinates, coincident stations, malformed routes, obstacle pause/resume, latched emergency stops, stale samples, read failures, mission timeout, motor watchdog expiration and timer wrap.

Integration checks verify the robot returns within 6 cm of the depot, advances through every route target, emits finite bounded commands, and commands zero for BLOCKED/COMPLETE/FAULT. The obstacle scenario must remain stationary for exactly 150 ticks and complete three seconds later than the normal scenario. Emergency stop and dropout must fault at the injected three-second mark with the correct reason.

Tests validate this model; they do not establish behavior on real hardware or worst-case execution time. Sensor noise, packet corruption, friction, wheel slip and acceleration limits are not modeled. There is no exhaustive proof that every possible coordinate layout completes within the fixed timeout.
