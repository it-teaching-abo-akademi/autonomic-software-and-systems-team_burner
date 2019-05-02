## Autonomic Software and Systems by team_burner

### Currently implemented
- Calculating direction and distance to the current destination.
- Navigation by waypoints. Able to navigate between two points on the map, but often takes some serious detours. The detours are a temporary measure due to the work on the navigator. Just a few lines of code needs to be changed to get the car to drive in a straight line to the next waypoint (like in e.g. milestone 1).
- Throttle that is adjusted according to the difference between current and desired speed.
- Progressive steering that turns the wheel more the bigger the difference is between the current and desired direction.
- Stopping at red lights.
- Following speed limits somewhat. More powerful cars might go a few km/h over the limit with the current formula for calculating throttle, but perhaps that only makes the AI more human... ;)

### In the works
- Using Dijkstra's algorithm with the map topology to determin the shortest path to the current destination while still following the traffic rules (i.e. driving on the right side of the road and not going offroad except when explicitly instructed, etc.).
