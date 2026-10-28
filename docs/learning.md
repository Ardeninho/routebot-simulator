# Development guide

## Explore the system

1. Run `make demo` and inspect the station order and route trace. Planned and driven distances differ because the controller accepts waypoints within a 6 cm radius.
2. Read `planner.c` and trace nearest insertion on four points. Cached distances eliminate repeated scans of all subtour members.
3. Read `controller.c` and follow the state transitions. Clearing an emergency-stop signal does not restart a latched fault; a new mission start is required.
4. Derive the wheel-speed equations from track width. Change the heading gain and compare completion time and path shape.
5. Inspect the HAL callbacks and inject a stale sensor timestamp. Run the regression tests to check that motion stops.

## Reproduce a demonstration

- Run `make demo` to generate the route comparison and trajectory.
- Run `./build/routebot estop` to inspect the final zero motor command and latched FAULT. Exit status 1 is expected.
- Run `make test` for the controller and simulator checks.
- Run `make sanitize` to check for memory and undefined-behavior errors.

## Planned extensions

1. Load station coordinates from a validated mission file.
2. Add wheel acceleration limits and sensor noise; measure arrival error and tuning sensitivity.
3. Add a serial packet protocol with sequence numbers, length validation and checksum tests.
4. Port the controller to a board or emulator and measure memory use and execution time.

Each extension should include a reproducible test and a design note. Hardware deployment and real-time deadline validation remain future work.
