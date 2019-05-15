## Autonomic Software and Systems by team_burner

### Currently implemented
- Calculating direction and distance to the current destination.
- Navigation by waypoints.
- Throttle that is adjusted according to the difference between current and desired speed. With a control update interval of 0.5 seconds the car must drive quite slowly. The throttle can be increased significantly if the interval is shortened to e.g. 0.05 or 0.02 seconds.
- Progressive steering that turns the wheel more the bigger the difference is between the current and desired direction.
- Stopping at red lights.
- Following speed limits.

### Needed
- Getting the car back on the path after a crash.
- Taking traffic lights better into account. At the moment the car occasionally stops on the pedestrian crossing, which is not so good.
- Adaptive steering. Currently the cars wobble a lot at higher speeds. UPDATE: the wobbling is nearly non-existent if the control update interval is shortened.
- Adaptive throttle so that all cars keep the target speed as instructed by the planner, no matter how powerful engine they have.
