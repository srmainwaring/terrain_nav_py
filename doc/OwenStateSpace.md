# OwenStateSpace Testing

Testing the OwenStateSpace for terrain planning, and checking the output of
the generated Dubins Path.


## Path

[0] [ 793.32, -385.61, 1620.89]; ter_alt: 1561.83, agl_alt:  59.06
[1] [ 165.96, -573.52, 1723.04]; ter_alt: 1615.26, agl_alt: 107.78
[2] [ 165.96, -573.52, 1723.04]; ter_alt: 1615.26, agl_alt: 107.78
[3] [-180.14, -313.81, 1809.65]; ter_alt: 1734.98, agl_alt:  74.67
[4] [-650.50, -801.49, 1914.26]; ter_alt: 1810.15, agl_alt: 104.11
[5] [-780.89, -459.97, 2018.86]; ter_alt: 1903.59, agl_alt: 115.28
[6] [-965.01, -131.75, 2123.47]; ter_alt: 2035.38, agl_alt:  88.09
[7] [-483.99,  370.81, 2199.74]; ter_alt: 2081.82, agl_alt: 117.92
[8] [-793.32,  385.61, 2253.40]; ter_alt: 2196.56, agl_alt:  56.84

Notes / issues:

- Repeated states: [1] and [2] in the solution vector
-  



## Appendix

```python
# Davos
start_lat = 46.8141348
start_lon = 9.8488310
goal_lat = 46.8201124
goal_lon = 9.8260916
grid_length_factor = 2.0

# settings
loiter_radius = 90.0
loiter_alt = 60.0
turning_radius = 90.0
climb_angle_rad = 0.15
max_altitude = 100.0
min_altitude = 40.0
time_budget = 10.0
resolution_m = 100.0
```

```bash
cd /Volumes/MacPro2_DV1/Code/ros2/jazzy/ros2-aerial ; /usr/bin/env /Users/rhys/.venv/ros2-3.13/bin/python /Users/rhys/.vscode/extensions/ms-python.debugpy-2025.5.2025031101-darwin
-x64/bundled/libs/debugpy/adapter/../../debugpy/launcher 61810 -- /Volumes/MacPro2_DV1/Code/ros2/jazzy/ros2-aerial/src/terrain_nav_py/tests/test_terrain_ompl_rrt_owen.py 
2025-04-04 11:18:41,861 DEBUG [__init__] Added a stderr logging handler to logger: terrain_nav_py
2025-04-04 11:18:41,861 DEBUG [test_terrain_ompl_rrt_owen] Added a stderr logging handler to logger: __main__
LOG_DEBUG
2025-04-04 11:18:41,861 DEBUG [test_terrain_ompl_rrt_owen] distance:       1856 m
2025-04-04 11:18:41,862 DEBUG [test_terrain_ompl_rrt_owen] bearing:        291.0 deg
2025-04-04 11:18:41,862 DEBUG [test_terrain_ompl_rrt_owen] east:           -1732 m
2025-04-04 11:18:41,862 DEBUG [test_terrain_ompl_rrt_owen] north:          665 m
2025-04-04 11:18:41,862 DEBUG [test_terrain_ompl_rrt_owen] grid_length:    3465 m
2025-04-04 11:18:41,862 DEBUG [test_terrain_ompl_rrt_owen] start_east:     866 m
2025-04-04 11:18:41,862 DEBUG [test_terrain_ompl_rrt_owen] start_north:    -333 m
2025-04-04 11:18:41,862 DEBUG [test_terrain_ompl_rrt_owen] goal_east:      -866 m
2025-04-04 11:18:41,862 DEBUG [test_terrain_ompl_rrt_owen] goal_north:     333 m
2025-04-04 11:18:41,873 DEBUG [test_terrain_ompl_rrt_owen] calculating distance-surface...
2025-04-04 11:18:42,272 DEBUG [terrain_ompl_rrt_owen] Upper bounds: (1732.258784510229, 1732.258784510229, 2667.9078381438344)
2025-04-04 11:18:42,272 DEBUG [terrain_ompl_rrt_owen] Lower bounds: (-1732.258784510229, -1732.258784510229, 1589.1368495719935)
2025-04-04 11:18:42,272 DEBUG [test_terrain_ompl_rrt_owen] Accept start position: [866.1293922551145, -332.70976400018816, 1620.8852799999927]
2025-04-04 11:18:42,273 DEBUG [test_terrain_ompl_rrt_owen] Accept goal position: [866.1293922551145, -332.70976400018816, 1620.8852799999927]
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] SetupProblem2
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] Configure problem
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] Clearing previous states
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] Setup default planner and objective
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] Setup terrain collition checking
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] Get lower bounds
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] Get upper bounds
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] Set bounds
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] Setup validity check resolution
2025-04-04 11:18:42,273 DEBUG [terrain_ompl_rrt_owen] Setup planner data 
2025-04-04 11:18:42,274 DEBUG [terrain_ompl_rrt_owen] Setting problem goals
2025-04-04 11:18:42,275 DEBUG [terrain_ompl_rrt_owen] re3_space.getBounds: low: (-1732.258784510229, -1732.258784510229, 1589.1368495719935)
2025-04-04 11:18:42,275 DEBUG [terrain_ompl_rrt_owen] re3_space.getBounds: high: (1732.258784510229, 1732.258784510229, 2667.9078381438344)
2025-04-04 11:18:42,275 DEBUG [terrain_ompl_rrt_owen] Running problem setup
Warning: RRTstar requires a state space with symmetric distance and symmetric interpolation.
         at line 101 in /Users/rhys/Code/ompl/ompl/src/ompl/geometric/planners/rrt/src/RRTstar.cpp
2025-04-04 11:18:42,275 DEBUG [terrain_ompl_rrt_owen] Get planner from problem
2025-04-04 11:18:42,275 DEBUG [terrain_ompl_rrt_owen] Planner range: 700.0
2025-04-04 11:18:42,275 DEBUG [test_terrain_ompl_rrt_owen] Resolution used: 0.028864047593291194
2025-04-04 11:18:42,275 DEBUG [terrain_ompl_rrt_owen] Run solver
Info:    RRTstar: Started planning with 10 states. Seeking a solution better than 0.00000.
Info:    RRTstar: Initial k-nearest value of 287
Info:    RRTstar: Found an initial solution with a cost of 5224.08 in 11301 iterations (813 vertices in the graph)
Info:    RRTstar: Created 826 new states. Checked 348453 rewire options. 2 goal states in tree. Final solution cost 4666.115
Info:    Solution found in 10.131934 seconds
2025-04-04 11:18:52,408 DEBUG [terrain_ompl_rrt_owen] Found exact solution!
2025-04-04 11:18:52,408 DEBUG [test_terrain_ompl_rrt_owen] [elevation]: [793.32, -385.61, 1620.89]; ter_alt: 1561.83, agl_alt: 59.06
2025-04-04 11:18:52,408 DEBUG [test_terrain_ompl_rrt_owen] [elevation]: [165.96, -573.52, 1723.04]; ter_alt: 1615.26, agl_alt: 107.78
2025-04-04 11:18:52,408 DEBUG [test_terrain_ompl_rrt_owen] [elevation]: [165.96, -573.52, 1723.04]; ter_alt: 1615.26, agl_alt: 107.78
2025-04-04 11:18:52,408 DEBUG [test_terrain_ompl_rrt_owen] [elevation]: [-180.14, -313.81, 1809.65]; ter_alt: 1734.98, agl_alt: 74.67
2025-04-04 11:18:52,408 DEBUG [test_terrain_ompl_rrt_owen] [elevation]: [-650.50, -801.49, 1914.26]; ter_alt: 1810.15, agl_alt: 104.11
2025-04-04 11:18:52,408 DEBUG [test_terrain_ompl_rrt_owen] [elevation]: [-780.89, -459.97, 2018.86]; ter_alt: 1903.59, agl_alt: 115.28
2025-04-04 11:18:52,408 DEBUG [test_terrain_ompl_rrt_owen] [elevation]: [-965.01, -131.75, 2123.47]; ter_alt: 2035.38, agl_alt: 88.09
2025-04-04 11:18:52,408 DEBUG [test_terrain_ompl_rrt_owen] [elevation]: [-483.99, 370.81, 2199.74]; ter_alt: 2081.82, agl_alt: 117.92
2025-04-04 11:18:52,408 DEBUG [test_terrain_ompl_rrt_owen] [elevation]: [-793.32, 385.61, 2253.40]; ter_alt: 2196.56, agl_alt: 56.84

from_state: 793.3, -385.6, 1620.9, 2.2
to_state:   166.0, -573.5, 1723.0, -2.9
path_type.phi: 0.00
path_type.deltaZ: 102.1524
path_type.numTurns: 0
path_type.path: DubinsPath[ type=LSR, length=1.34148+6.10881+0.152136=7.60243, reverse=0 ]
path_type.path.type: DUBINS_LEFT, DUBINS_STRAIGHT. DUBINS_RIGHT
path_type.path.length: 1.3415, 6.1088, 0.1521
path_type.path.reverse: False

from_state: 166.0, -573.5, 1723.0, -2.9
to_state:   166.0, -573.5, 1723.0, -2.9
path_type.phi: 0.00
path_type.deltaZ: 0.0000
path_type.numTurns: 0
path_type.path: DubinsPath[ type=LSL, length=0+0+0=0, reverse=0 ]
path_type.path.type: DUBINS_LEFT, DUBINS_STRAIGHT. DUBINS_LEFT
path_type.path.length: 0.0000, 0.0000, 0.0000
path_type.path.reverse: False

from_state: 166.0, -573.5, 1723.0, -2.9
to_state:   -180.1, -313.8, 1809.7, -2.5
path_type.phi: 0.42
path_type.deltaZ: 86.6137
path_type.numTurns: 0
path_type.path: DubinsPath[ type=RSL, length=2.08225+1.81312+2.05173=5.9471, reverse=0 ]
path_type.path.type: DUBINS_RIGHT, DUBINS_STRAIGHT. DUBINS_LEFT
path_type.path.length: 2.0823, 1.8131, 2.0517
path_type.path.reverse: False

from_state: -180.1, -313.8, 1809.7, -2.5
to_state:   -650.5, -801.5, 1914.3, -2.4
path_type.phi: 0.92
path_type.deltaZ: 104.6067
path_type.numTurns: 0
path_type.path: DubinsPath[ type=RSR, length=0.84083+5.93041+0=6.77124, reverse=0 ]
path_type.path.type: DUBINS_RIGHT, DUBINS_STRAIGHT. DUBINS_RIGHT
path_type.path.length: 0.8408, 5.9304, 0.0000
path_type.path.reverse: False

from_state: -650.5, -801.5, 1914.3, -2.4
to_state:   -780.9, -460.0, 2018.9, 1.4
path_type.phi: 0.62
path_type.deltaZ: 104.6067
path_type.numTurns: 0
path_type.path: DubinsPath[ type=RSR, length=3.11279+3.95902+1.77636e-15=7.07181, reverse=0 ]
path_type.path.type: DUBINS_RIGHT, DUBINS_STRAIGHT. DUBINS_RIGHT
path_type.path.length: 3.1128, 3.9590, 0.0000
path_type.path.reverse: False

from_state: -780.9, -460.0, 2018.9, 1.4
to_state:   -965.0, -131.7, 2123.5, 1.0
path_type.phi: 2.64
path_type.deltaZ: 104.6067
path_type.numTurns: 0
path_type.path: DubinsPath[ type=RSL, length=3.11872+1.78816+0.145191=5.05207, reverse=0 ]
path_type.path.type: DUBINS_RIGHT, DUBINS_STRAIGHT. DUBINS_LEFT
path_type.path.length: 3.1187, 1.7882, 0.1452
path_type.path.reverse: False

from_state: -965.0, -131.7, 2123.5, 1.0
to_state:   -484.0, 370.8, 2199.7, 0.8
path_type.phi: 0.00
path_type.deltaZ: 76.2654
path_type.numTurns: 0
path_type.path: DubinsPath[ type=RSR, length=0.223457+7.50802+0=7.73148, reverse=0 ]
path_type.path.type: DUBINS_RIGHT, DUBINS_STRAIGHT. DUBINS_RIGHT
path_type.path.length: 0.2235, 7.5080, 0.0000
path_type.path.reverse: False

from_state: -484.0, 370.8, 2199.7, 0.8
to_state:   -793.3, 385.6, 2253.4, 2.2
path_type.phi: 0.00
path_type.deltaZ: 53.6642
path_type.numTurns: 0
path_type.path: DubinsPath[ type=RLR, length=0.0641659+3.98803+2.52882=6.58102, reverse=0 ]
path_type.path.type: DUBINS_RIGHT, DUBINS_LEFT. DUBINS_RIGHT
path_type.path.length: 0.0642, 3.9880, 2.5288
path_type.path.reverse: False
path len: 9
path len: 1000
2025-04-04 11:18:53.855 Python[9955:4047783] +[IMKClient subclass]: chose IMKClient_Modern
2025-04-04 11:18:53.855 Python[9955:4047783] +[IMKInputSession subclass]: chose IMKInputSession_Modern
```