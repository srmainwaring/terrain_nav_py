# MAVLink Mission

Generate a MAVLink Mission from a Path

### Segment types

- Turn Left
- Turn Right
- Straight
- Loiter to Alt

### MAVLink commands

- MAV_CMD_NAV_WAYPOINT (16)
  - 

- MAV_CMD_NAV_LOITER_TURNS (18)
  - param1: turns
  - param2: empty
  - param3: radius
  - param4: exit: 0 next waypoint path intersects, 1 for next waypoint path is tangent

- MAV_CMD_NAV_LOITER_TO_ALT (31)
  - param1: turns
  - param2: empty
  - param3: radius
  - param4: exit: 0 next waypoint path intersects, 1 for next waypoint path is tangent


### Example:

