## Autonomic Software and Systems by team_burner

### Currently implemented
- Calculating direction and distance to the current destination.
- Navigation by waypoints. Able to navigate between two points on the map, but often takes some serious detours.
- Throttle that is adjusted according to the difference between current and desired speed.
- Progressive steering that turns the wheel more the bigger the difference is between the current and desired direction.
- Stopping at red lights.
- Following speed limits somewhat. More powerful cars might go a few km/h over the limit with the current formula for calculating throttle, but perhaps that only makes the AI more human... ;)

### Needed
- Getting the car back on the path after a crash.
- Taking traffic lights better into account. At the moment the car often stops on the pedestrian crossing, which is quite bad.
- Adaptive steering. Currently the casr wobble a lot at higher speeds.
- Adaptive throttle so that all cars keep the target speed as instructed by the planner, no matter how powerful engine they have.
