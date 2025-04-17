# BSD 3-Clause License
#
# Copyright (c) 2025, Rhys Mainwaring
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import logging
import math

from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou

from MAVProxy.modules.lib import mp_util

from pymavlink.rotmat import Vector3

from terrain_nav_py import add_stderr_logger

from terrain_nav_py.path import Path

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace

from terrain_nav_py.grid_map import GridMapSRTM

from terrain_nav_py.path import Path
from terrain_nav_py.path_segment import PathSegment

from terrain_nav_py.terrain_map import TerrainMap

from terrain_nav_py.terrain_ompl_rrt import TerrainOmplRrt

log = logging.getLogger(__name__)


def add_test_stderr_logger(level: int = logging.DEBUG) -> logging.StreamHandler:
    logger = logging.getLogger(__name__)
    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)s [%(module)s] %(message)s")
    )
    logger.addHandler(handler)
    logger.setLevel(level)
    logger.debug(f"Added a stderr logging handler to logger: {__name__}")
    return handler


def test_terrain_ompl_rrt():
    # set the RNG seed for repeatable outcomes
    ou.RNG.setSeed(1)

    # configure loggers
    add_stderr_logger()
    add_test_stderr_logger()
    ou.setLogLevel(ou.LogLevel.LOG_DEBUG)
    print(ou.getLogLevel())

    # Foothills Community Park, Colorado
    start_lat = 40.056671934301086
    start_lon = -105.28785817446858

    # Buckingham Picnic Area, Colorado
    goal_lat = 40.11188249790071
    goal_lon = -105.30681932208977
    grid_length_factor = 1.2

    # Ward, Colorado
    # goal_lat = 40.072504162423655
    # goal_lon = -105.50885876436985

    # Colorado Mountain Ranch, Colorado
    # goal_lat = 40.061037962756885
    # goal_lon = -105.41560711209344

    # Davos
    # start_lat = 46.8141348
    # start_lon = 9.8488310
    # goal_lat = 46.8201124
    # goal_lon = 9.8260916
    # grid_length_factor = 5.0

    # Brecon, Storey Arms to Pen-y-fan
    # example includes a spiral when
    # turning_radius = 60
    # max_alt = 100
    # min_alt = 60
    # climb_angle_rad = 0.10
    start_lat = 51.86949481854276
    start_lon = -3.475730001422848
    goal_lat = 51.883459907192325
    goal_lon = -3.4367987405967733
    grid_length_factor = 2.0

    distance = mp_util.gps_distance(start_lat, start_lon, goal_lat, goal_lon)
    bearing_deg = mp_util.gps_bearing(start_lat, start_lon, goal_lat, goal_lon)
    bearing_rad = math.radians(bearing_deg)

    east = distance * math.sin(bearing_rad)
    north = distance * math.cos(bearing_rad)
    log.debug(f"distance:       {distance:.0f} m")
    log.debug(f"bearing:        {bearing_deg:.1f} deg")
    log.debug(f"east:           {east:.0f} m")
    log.debug(f"north:          {north:.0f} m")

    # set map size and centre
    (map_lat, map_lon) = mp_util.gps_offset(
        start_lat, start_lon, 0.5 * east, 0.5 * north
    )
    log.debug(f"map_lat:        {map_lat}")
    log.debug(f"map_lon:        {map_lon}")

    grid_length = grid_length_factor * max(math.fabs(east), math.fabs(north))
    log.debug(f"grid_length:    {grid_length:.0f} m")

    start_east = -0.5 * east
    start_north = -0.5 * north
    goal_east = 0.5 * east
    goal_north = 0.5 * north
    log.debug(f"start_east:     {start_east:.0f} m")
    log.debug(f"start_north:    {start_north:.0f} m")
    log.debug(f"goal_east:      {goal_east:.0f} m")
    log.debug(f"goal_north:     {goal_north:.0f} m")

    # settings
    loiter_radius = 60.0
    loiter_alt = 60.0
    turning_radius = 60.0
    climb_angle_rad = 0.1
    max_altitude = 100.0
    min_altitude = 60.0
    time_budget = 20.0
    resolution_m = 100.0

    # create map
    grid_map = GridMapSRTM(map_lat=map_lat, map_lon=map_lon)
    grid_map.setGridSpacing(30)
    grid_map.setGridLength(grid_length)

    # set up distance layer (may take a while..)
    log.debug(f"calculating distance-surface...")
    # grid_map.addLayerDistanceTransform(surface_distance=min_altitude)

    terrain_map = TerrainMap()
    terrain_map.setGridMap(grid_map)

    # create planner manager
    da_space = DubinsAirplaneStateSpace(
        turningRadius=turning_radius, gam=climb_angle_rad
    )
    planner_mgr = TerrainOmplRrt(da_space)
    planner_mgr.setMap(terrain_map)
    planner_mgr.setAltitudeLimits(max_altitude=max_altitude, min_altitude=min_altitude)
    planner_mgr.setBoundsFromMap(terrain_map.getGridMap())

    # set start and goal positions
    start_pos = [start_east, start_north, loiter_alt]
    goal_pos = [goal_east, goal_north, loiter_alt]

    # adjust the start and goal altitudes
    start_pos[2] += grid_map.atPosition("elevation", start_pos)
    goal_pos[2] += grid_map.atPosition("elevation", goal_pos)

    # TODO: check the start and goal states are valid. This requires a
    #       correctly calculated distance surface: a surface where at each
    # point a circle of radius r will not intersect with the elevation layer.
    is_start_valid = TerrainOmplRrt.validatePosition(grid_map, start_pos, loiter_radius)
    if not is_start_valid:
        log.debug(f"Invalid start position: {start_pos}")
    else:
        log.debug(f"Accept start position: {start_pos}")
    is_goal_valid = TerrainOmplRrt.validatePosition(grid_map, start_pos, loiter_radius)
    if not is_goal_valid:
        log.debug(f"Invalid goal position: {goal_pos}")
    else:
        log.debug(f"Accept goal position: {start_pos}")

    # NOTE: if the time budget is insufficient, the solution tree may not
    #       include a goal state, and an approximate solution will be found.

    # PLANNER_MODE.GLOBAL
    # set up problem from start and goal positions and start loiter radius
    planner_mgr.setupProblem2(start_pos, goal_pos, turning_radius)

    # Adjust validity checking resolution as needed. This is expressed as
    # a fraction of the spaces extent.
    problem = planner_mgr.getProblemSetup()
    resolution_requested = resolution_m / grid_length
    problem.setStateValidityCheckingResolution(resolution_requested)
    si = problem.getSpaceInformation()
    resolution_used = si.getStateValidityCheckingResolution()
    log.debug(f"Resolution used: {resolution_used}")

    candidate_path = Path()
    planner_mgr.Solve1(time_budget=time_budget, path=candidate_path)

    # PLANNER_MODE.EMERGENCY_ABORT
    # set up problem start position and velocity and rally points
    # planner_mgr.setupProblem4(start_pos, start_vel, rally_points)
    # planner_mgr_solution_path = Path()
    # planner_mgr.Solve(time_budget=1.0, path=planner_solution_path)

    # PLANNER_MODE.RETURN
    # set up problem start position and velocity and home position and radius
    # planner_mgr.setupProblem3(start_pos, start_vel, home_pos, home_pos_radius)
    # planner_mgr_solution_path = Path()
    # planner_mgr.Solve(time_budget=1.0, path=planner_solution_path)

    # check states are all above the min_altitude
    solution_path = planner_mgr._problem_setup.getSolutionPath()
    states = solution_path.getStates()
    for state in states:
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        (x, y, z, yaw) = da_state.getXYZYaw()
        layer = "elevation"
        elevation = grid_map.atPosition(layer, da_state.getXYZ())
        log.debug(
            f"[{layer}]: "
            f"[{x:.2f}, {y:.2f}, {z:.2f}]; "
            f"ter_alt: {elevation:.2f}, agl_alt: {(z - elevation):.2f}"
        )

    # More information... from og.SimpleSetup
    problem = planner_mgr.getProblemSetup()

    problem.getGoal()
    last_comp_time = problem.getLastPlanComputationTime()
    status = problem.getLastPlannerStatus()
    last_simp_time = problem.getLastSimplificationTime()
    oo = problem.getOptimizationObjective()
    planner = problem.getPlanner()
    problem_definition = problem.getProblemDefinition()
    solution_path = problem.getSolutionPath()
    planner_name = problem.getSolutionPlannerName()
    si = problem.getSpaceInformation()
    vc = problem.getStateValidityChecker()

    planner_data = ob.PlannerData(si)
    problem.getPlannerData(planner_data)
    log.debug(f"numVertices: {planner_data.numVertices()}")
    log.debug(f"numEdges: {planner_data.numEdges()}")

    # only display plots if run as script
    if __name__ == "__main__":
        plot_path(start_pos, goal_pos, loiter_radius, candidate_path, states, grid_map)


def test_terrain_ompl_rrt_solution_path_to_path():
    """
    loiter_radius = 60.0
    climb_angle_rad = 0.1
    max_altitude = 100.0
    min_altitude = 60.0

    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] distance:       3094 m
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] bearing:        59.8 deg
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] east:           2675 m
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] north:          1555 m
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] map_lat:        51.87647736286754
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] map_lon:        -3.4562658824495816
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] grid_length:    5351 m
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] start_east:     -1338 m
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] start_north:    -777 m
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] goal_east:      1338 m
    2025-04-17 17:27:57,615 DEBUG [test_terrain_ompl_rrt] goal_north:     777 m

    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] Path has 11 states
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] state[0]:    [-1319.2087, -834.3522, 496.3328; -2.8274]
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] state[1]:    [-1509.0698, -277.3968, 557.1601; 2.1498]
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_RSL
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_RIGHT DUBINS_STRAIGHT DUBINS_LEFT
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] dubins.len: 1.676 8.380 0.370
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_LOW
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A11
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] dubins.ks:   0
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] segs[0]:     x: -1319.209, y: -834.352, z: 496.333, yaw: -2.827
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] segs[1]:     x: -1319.209, y: -834.352, z: 496.333, yaw: -2.827
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] segs[2]:     x: -1396.449, y: -789.716, z: 506.112, yaw: 1.779
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] segs[3]:     x: -1396.449, y: -789.716, z: 506.112, yaw: 1.779
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] segs[4]:     x: -1500.591, y: -297.802, z: 554.999, yaw: 1.779
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] segs[5]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:53,824 DEBUG [terrain_ompl_rrt] total_length: 10.427002385674154
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] state[1]:    [-1509.0698, -277.3968, 557.1601; 2.1498]
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] state[2]:    [-1376.9522, 29.2846, 617.5511; 1.7377]
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_RSL
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_RIGHT DUBINS_STRAIGHT DUBINS_LEFT
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] dubins.len: 1.135 3.935 0.723
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_MEDIUM
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A11
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] dubins.ks:   1
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] segs[0]:     x: -1509.070, y: -277.397, z: 557.160, yaw: 2.150
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] segs[1]:     x: -1509.070, y: -277.397, z: 588.580, yaw: 2.150
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] segs[2]:     x: -1509.808, y: -212.889, z: 594.257, yaw: 1.015
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] segs[3]:     x: -1509.808, y: -212.889, z: 594.257, yaw: 1.015
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] segs[4]:     x: -1385.160, y: -12.360, z: 613.935, yaw: 1.015
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] segs[5]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:53,860 DEBUG [terrain_ompl_rrt] total_length: 12.076638929545622
    2025-04-17 16:50:53,908 DEBUG [terrain_ompl_rrt] state[2]:    [-1376.9522, 29.2846, 617.5511; 1.7377]
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] state[3]:    [-1398.0118, 727.1960, 666.9492; 1.6002]
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_RSR
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_RIGHT DUBINS_STRAIGHT DUBINS_RIGHT
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] dubins.len: 0.138 11.500 0.000
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_LOW
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A14
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] dubins.ks:   0
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] segs[0]:     x: -1376.952, y: 29.285, z: 617.551, yaw: 1.738
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] segs[1]:     x: -1376.952, y: 29.285, z: 617.551, yaw: 1.738
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] segs[2]:     x: -1377.761, y: 37.493, z: 618.135, yaw: 1.600
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] segs[3]:     x: -1377.761, y: 37.493, z: 618.135, yaw: 1.600
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] segs[4]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] segs[5]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:53,909 DEBUG [terrain_ompl_rrt] total_length: 11.637580776286727
    2025-04-17 16:50:53,932 DEBUG [terrain_ompl_rrt] state[3]:    [-1398.0118, 727.1960, 666.9492; 1.6002]
    2025-04-17 16:50:53,932 DEBUG [terrain_ompl_rrt] state[4]:    [-993.7344, 1293.9982, 704.4269; 0.9327]
    2025-04-17 16:50:53,932 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_RSR
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_RIGHT DUBINS_STRAIGHT DUBINS_RIGHT
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] dubins.len: 0.667 10.983 0.000
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_LOW
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A14
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] dubins.ks:   0
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] segs[0]:     x: -1398.012, y: 727.196, z: 666.949, yaw: 1.600
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] segs[1]:     x: -1398.012, y: 727.196, z: 666.949, yaw: 1.600
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] segs[2]:     x: -1386.233, y: 764.695, z: 669.096, yaw: 0.933
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] segs[3]:     x: -1386.233, y: 764.695, z: 669.096, yaw: 0.933
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] segs[4]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] segs[5]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:53,933 DEBUG [terrain_ompl_rrt] total_length: 11.649933493305893
    2025-04-17 16:50:53,956 DEBUG [terrain_ompl_rrt] state[4]:    [-993.7344, 1293.9982, 704.4269; 0.9327]
    2025-04-17 16:50:53,956 DEBUG [terrain_ompl_rrt] state[5]:    [-513.9431, 1532.9495, 689.4346; 2.3127]
    2025-04-17 16:50:53,956 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_RSL
    2025-04-17 16:50:53,956 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_RIGHT DUBINS_STRAIGHT DUBINS_LEFT
    2025-04-17 16:50:53,956 DEBUG [terrain_ompl_rrt] dubins.len: 0.657 7.274 2.037
    2025-04-17 16:50:53,956 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_LOW
    2025-04-17 16:50:53,956 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A12
    2025-04-17 16:50:53,956 DEBUG [terrain_ompl_rrt] dubins.ks:   0
    2025-04-17 16:50:53,957 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:53,957 DEBUG [terrain_ompl_rrt] segs[0]:     x: -993.734, y: 1293.998, z: 704.427, yaw: 0.933
    2025-04-17 16:50:53,957 DEBUG [terrain_ompl_rrt] segs[1]:     x: -993.734, y: 1293.998, z: 704.427, yaw: 0.933
    2025-04-17 16:50:53,957 DEBUG [terrain_ompl_rrt] segs[2]:     x: -961.854, y: 1315.999, z: 703.438, yaw: 0.275
    2025-04-17 16:50:53,957 DEBUG [terrain_ompl_rrt] segs[3]:     x: -961.854, y: 1315.999, z: 703.438, yaw: 0.275
    2025-04-17 16:50:53,957 DEBUG [terrain_ompl_rrt] segs[4]:     x: -541.860, y: 1434.669, z: 692.499, yaw: 0.275
    2025-04-17 16:50:53,957 DEBUG [terrain_ompl_rrt] segs[5]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:53,957 DEBUG [terrain_ompl_rrt] total_length: 9.968628157928265
    2025-04-17 16:50:53,987 DEBUG [terrain_ompl_rrt] state[5]:    [-513.9431, 1532.9495, 689.4346; 2.3127]
    2025-04-17 16:50:53,987 DEBUG [terrain_ompl_rrt] state[6]:    [-95.2192, 1252.5827, 751.8279; -0.6693]
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_RSL
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_RIGHT DUBINS_STRAIGHT DUBINS_LEFT
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] dubins.len: 3.145 7.995 0.163
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_LOW
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A24
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] dubins.ks:   0
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] segs[0]:     x: -513.943, y: 1532.949, z: 689.435, yaw: 2.313
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] segs[1]:     x: -513.943, y: 1532.949, z: 689.435, yaw: 2.313
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] segs[2]:     x: -425.352, y: 1613.891, z: 706.795, yaw: -0.832
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] segs[3]:     x: -425.352, y: 1613.891, z: 706.795, yaw: -0.832
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] segs[4]:     x: -102.352, y: 1259.237, z: 750.929, yaw: -0.832
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] segs[5]:     x: -95.219, y: 1252.583, z: 751.828, yaw: -0.669
    2025-04-17 16:50:53,988 DEBUG [terrain_ompl_rrt] total_length: 11.302470582915976
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] state[6]:    [-95.2192, 1252.5827, 751.8279; -0.6693]
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] state[7]:    [354.7171, 720.7249, 819.4522; -0.8704]
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_RSR
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_RIGHT DUBINS_STRAIGHT DUBINS_RIGHT
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] dubins.len: 0.201 11.411 0.000
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_LOW
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A14
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] dubins.ks:   0
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] segs[0]:     x: -95.219, y: 1252.583, z: 751.828, yaw: -0.669
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] segs[1]:     x: -95.219, y: 1252.583, z: 751.828, yaw: -0.669
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] segs[2]:     x: -86.572, y: 1244.200, z: 752.999, yaw: -0.870
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] segs[3]:     x: -86.572, y: 1244.200, z: 752.999, yaw: -0.870
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] segs[4]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] segs[5]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:54,022 DEBUG [terrain_ompl_rrt] total_length: 11.612097969419427
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] state[7]:    [354.7171, 720.7249, 819.4522; -0.8704]
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] state[8]:    [659.1998, 313.5875, 843.8850; 0.5264]
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_RSL
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_RIGHT DUBINS_STRAIGHT DUBINS_LEFT
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] dubins.len: 0.179 7.233 1.576
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_LOW
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A11
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] dubins.ks:   0
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] segs[0]:     x: 354.717, y: 720.725, z: 819.452, yaw: -0.870
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] segs[1]:     x: 354.717, y: 720.725, z: 819.452, yaw: -0.870
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] segs[2]:     x: 360.872, y: 711.935, z: 819.939, yaw: -1.049
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] segs[3]:     x: 360.872, y: 711.935, z: 819.939, yaw: -1.049
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] segs[4]:     x: 577.025, y: 335.582, z: 839.601, yaw: -1.049
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] segs[5]:     x: 659.200, y: 313.588, z: 843.885, yaw: 0.526
    2025-04-17 16:50:54,046 DEBUG [terrain_ompl_rrt] total_length: 8.988424647444615
    2025-04-17 16:50:54,073 DEBUG [terrain_ompl_rrt] state[8]:    [659.1998, 313.5875, 843.8850; 0.5264]
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] state[9]:    [741.6659, 591.6972, 912.2411; 1.2548]
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_LSR
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_LEFT DUBINS_STRAIGHT DUBINS_RIGHT
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] dubins.len: 0.823 3.995 0.095
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_HIGH
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A44
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] dubins.ks:   1
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] segs[0]:     x: 659.200, y: 313.588, z: 843.885, yaw: 0.526
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] segs[1]:     x: 659.200, y: 313.588, z: 882.660, yaw: 0.526
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] segs[2]:     x: 687.594, y: 352.310, z: 887.616, yaw: 1.350
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] segs[3]:     x: 687.594, y: 352.310, z: 887.616, yaw: 1.350
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] segs[4]:     x: 740.154, y: 586.205, z: 911.669, yaw: 1.350
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] segs[5]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
    2025-04-17 16:50:54,074 DEBUG [terrain_ompl_rrt] total_length: 11.354685123522213
    2025-04-17 16:50:54,119 DEBUG [terrain_ompl_rrt] state[9]:    [741.6659, 591.6972, 912.2411; 1.2548]
    2025-04-17 16:50:54,119 DEBUG [terrain_ompl_rrt] state[10]:    [1356.2907, 720.2254, 941.3644; 0.3142]
    2025-04-17 16:50:54,119 DEBUG [terrain_ompl_rrt] dubins.idx:  TYPE_RSL
    2025-04-17 16:50:54,119 DEBUG [terrain_ompl_rrt] dubins.type: DUBINS_RIGHT DUBINS_STRAIGHT DUBINS_LEFT
    2025-04-17 16:50:54,119 DEBUG [terrain_ompl_rrt] dubins.len: 1.102 9.397 0.162
    2025-04-17 16:50:54,119 DEBUG [terrain_ompl_rrt] dubins.alt:  ALT_CASE_LOW
    2025-04-17 16:50:54,119 DEBUG [terrain_ompl_rrt] dubins.cls:  CLASS_A11
    2025-04-17 16:50:54,119 DEBUG [terrain_ompl_rrt] dubins.ks:   0
    2025-04-17 16:50:54,119 DEBUG [terrain_ompl_rrt] dubins.ke:   0
    2025-04-17 16:50:54,120 DEBUG [terrain_ompl_rrt] segs[0]:     x: 741.666, y: 591.697, z: 912.241, yaw: 1.255
    2025-04-17 16:50:54,120 DEBUG [terrain_ompl_rrt] segs[1]:     x: 741.666, y: 591.697, z: 912.241, yaw: 1.255
    2025-04-17 16:50:54,120 DEBUG [terrain_ompl_rrt] segs[2]:     x: 789.582, y: 632.354, z: 915.252, yaw: 0.152
    2025-04-17 16:50:54,120 DEBUG [terrain_ompl_rrt] segs[3]:     x: 789.582, y: 632.354, z: 915.252, yaw: 0.152
    2025-04-17 16:50:54,120 DEBUG [terrain_ompl_rrt] segs[4]:     x: 1346.862, y: 717.985, z: 940.923, yaw: 0.152
    2025-04-17 16:50:54,120 DEBUG [terrain_ompl_rrt] segs[5]:     x: 1356.291, y: 720.225, z: 941.364, yaw: 0.314
    2025-04-17 16:50:54,120 DEBUG [terrain_ompl_rrt] total_length: 10.66101320450275

    """
    # configure loggers
    add_stderr_logger()
    add_test_stderr_logger()
    ou.setLogLevel(ou.LogLevel.LOG_DEBUG)
    print(ou.getLogLevel())

    # inputs
    loiter_radius = 60.0
    climb_angle_rad = 0.1
    max_altitude = 100.0
    min_altitude = 60.0
    map_lat = 51.87647736286754
    map_lon = -3.4562658824495816
    grid_spacing = 30
    grid_length = 5351.0
    start_east = -1338.0
    start_north = -777.0
    goal_east = 1338.0
    goal_north = 777.0

    # states from planner solution
    solution_states = [
        [-1319.2087, -834.3522, 496.3328, -2.8274],
        [-1509.0698, -277.3968, 557.1601, 2.1498],
        [-1376.9522, 29.2846, 617.5511, 1.7377],
        [-1398.0118, 727.1960, 666.9492, 1.6002],
        [-993.7344, 1293.9982, 704.4269, 0.9327],
        [-513.9431, 1532.9495, 689.4346, 2.3127],
        [-95.2192, 1252.5827, 751.8279, -0.6693],
        [354.7171, 720.7249, 819.4522, -0.8704],
        [659.1998, 313.5875, 843.8850, 0.5264],
        [741.6659, 591.6972, 912.2411, 1.2548],
        [1356.2907, 720.2254, 941.3644, 0.3142],
    ]

    # create terrain map
    grid_map = GridMapSRTM(map_lat, map_lon)
    grid_map.setGridSpacing(grid_spacing)
    grid_map.setGridLength(grid_length)

    terrain_map = TerrainMap()
    terrain_map.setGridMap(grid_map)

    # create planner manager
    da_space = DubinsAirplaneStateSpace(
        turningRadius=loiter_radius, gam=climb_angle_rad
    )
    planner_mgr = TerrainOmplRrt(da_space)
    planner_mgr.setMap(terrain_map)
    planner_mgr.setAltitudeLimits(max_altitude, min_altitude)
    planner_mgr.setBoundsFromMap(terrain_map.getGridMap())

    # set start and goal
    start_pos = [start_east, start_north, min_altitude]
    goal_pos = [goal_east, goal_north, min_altitude]

    # adjust start and goal altitudes
    start_pos[2] += grid_map.atPosition("elevation", start_pos)
    goal_pos[2] += grid_map.atPosition("elevation", goal_pos)

    # set up problem from start and goal positions and start loiter radius
    planner_mgr.setupProblem2(start_pos, goal_pos, loiter_radius)

    # initialise an empty solution path
    problem = planner_mgr.getProblemSetup()
    si = problem.getSpaceInformation()
    solution_path = og.PathGeometric(si)

    # add states
    for s in solution_states:
        state = ob.State(da_space)
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        da_state.setXYZYaw(s[0], s[1], s[2], s[3])
        solution_path.append(state())

    trajectory_segments = Path()
    planner_mgr.solutionPathToPath(solution_path, trajectory_segments)

    states = solution_path.getStates()

    # Make / use a class for mission items
    # 0	  0	0	16	0.0	  0.0	  0.0	   0.0 51.86949482 -3.475730	 435.00 1
    # 1	  0	0	31	1.0	-60.0	  0.0	   1.0 51.86898184 -3.4742518  500.90 1

    mission_items = []
    wp_num = 0
    for i, segment in enumerate(trajectory_segments._segments):
        # print(f"[{i}] first:\n{segment.first_state()}")
        # print(f"[{i}] last:\n{segment.last_state()}")
        position3 = segment.first_state().position
        tangent3 = (segment.first_state().velocity).normalized()
        curvature = segment.curvature
        position2 = Vector3(position3.x, position3.y, 0.0)
        tangent2 = Vector3(tangent3.x, tangent3.y, 0.0)


        (start_lat, start_lon) = mp_util.gps_offset(
            map_lat, map_lon, position3.x, position3.y
        )
        start_alt = position3.z

        (end_lat, end_lon) = mp_util.gps_offset(
            map_lat,
            map_lon,
            segment.last_state().position.x,
            segment.last_state().position.y,
        )
        end_alt = segment.last_state().position.z

        # waypoint number - should also include home position
        sys_id = 0
        cmp_id = 0
        if curvature != 0.0:
            # do not add a loiter for short turns
            length = segment.get_length()
            radius = 1.0 / math.fabs(curvature)
            phi = length / radius
            if phi < 0.5 * math.pi:
                continue

            arc_centre = PathSegment.get_arc_centre(position2, tangent2, curvature)
            (cen_lat, cen_lon) = mp_util.gps_offset(
                map_lat,
                map_lon,
                arc_centre.x,
                arc_centre.y,
            )

            # print(f"[{i}]\ncentre:    {arc_centre}")
            # print(f"curvature: {curvature:.4f}")

            # mission item MAV_CMD_NAV_LOITER_TO_ALT (31)
            mav_id = 31
            p1 = 1 # heading required: 0: no, 1: yes 
            p2 = -1.0 / curvature # radius: > 0 loiter CW, < 0 loiter CCW
            p3 = 0
            p4 = 1 # loiter exit location: 1: line between exit and next wp.  
            p5 = cen_lat
            p6 = cen_lon
            p7 = end_alt
            p8 = 1 # abs alt
            mission_item = (
                f"{wp_num} {sys_id} {cmp_id} {mav_id} "
                f"{p1} {p2} {p3} {p4} {p5} {p6} {p7} {p8}"
            )
            print(mission_item)
            mission_items.append(mission_item)
            wp_num += 1

        # mission item MAV_CMD_NAV_WAYPOINT (16)
        mav_id = 16
        p1 = 0 # hold
        p2 = 0 # accept radius
        p3 = 0 # pass radius
        p4 = 0 # yaw at wapypoint 
        p5 = end_lat
        p6 = end_lon
        p7 = end_alt
        p8 = 1 # abs alt
        mission_item = (
            f"{wp_num} {sys_id} {cmp_id} {mav_id} "
            f"{p1} {p2} {p3} {p4} {p5} {p6} {p7} {p8}"
        )
        print(mission_item)
        mission_items.append(mission_item)
        wp_num += 1

    if __name__ == "__main__":
        plot_path(
            start_pos, goal_pos, loiter_radius, trajectory_segments, states, grid_map
        )


def plot_path(
    start_pos, goal_pos, loiter_radius, path=None, states=None, grid_map=None
):
    import matplotlib.pyplot as plt
    import numpy as np

    def plot_circle(ax, position, radius, label=""):
        num_step = 50
        theta = np.linspace(-math.pi, math.pi, num_step, endpoint=False)
        x = position[0] + radius * np.cos(theta)
        y = position[1] + radius * np.sin(theta)
        z = position[2] * np.ones(len(theta))
        ax.scatter(x, y, z, linestyle="solid", marker=".", s=1, c="blue")
        ax.text(position[0] + 1.5 * radius, position[1], position[2], label)

    def plot_path(ax, path):
        position = path.position()
        position = np.array(position)
        # check the state vector is not empty
        if position.size == 0:
            return
        x = np.array([p.x for p in position])
        y = np.array([p.y for p in position])
        z = np.array([p.z for p in position])
        ax.scatter(x, y, z, linestyle="solid", marker=".", s=1, c="green")

        # plot velocity vectors along the path
        velocity = path.velocity()
        velocity = np.array(velocity)

        # log.debug(f"position.shape: {position.shape}")
        # log.debug(f"velocity.shape: {velocity.shape}")

        scale = 0.25 * loiter_radius
        stride = 10
        vx = np.array([v.x for v in velocity])
        vy = np.array([v.y for v in velocity])
        vz = np.array([v.z for v in velocity])
        u = scale * vx
        v = scale * vy
        w = scale * vz
        ax.quiver(
            x[::stride],
            y[::stride],
            z[::stride],
            u[::stride],
            v[::stride],
            w[::stride],
            color="blue",
        )

    def plot_state(ax, state, label=""):
        position = TerrainOmplRrt.dubinsairplanePosition(state)
        yaw = TerrainOmplRrt.dubinsairplaneYaw(state)
        x = position[0]
        y = position[1]
        z = position[2]
        ax.scatter(x, y, z, marker="v", s=48, c="red")
        ax.text(position[0], position[1], position[2], label)

        # plot tangent
        scale = 0.5 * loiter_radius
        u = scale * np.cos(yaw)
        v = scale * np.sin(yaw)
        w = 0.0
        ax.quiver(x, y, z, u, v, w, color="red")

    def plot_states(ax, states):
        for i, state in enumerate(states):
            plot_state(ax, state, f"state{i}")

    def plot_terrain(ax, grid_map):
        # World frame is ENU, (x, y) = (east, north)
        length = grid_map.getLength()
        length_x = length[0]
        length_y = length[1]
        grid_spacing = 30
        num_step = int(length_x / grid_spacing)
        x = np.linspace(-0.5 * length_x, 0.5 * length_x, num_step)
        num_step = int(length_y / grid_spacing)
        y = np.linspace(-0.5 * length_y, 0.5 * length_y, num_step)
        x_grid, y_grid = np.meshgrid(x, y)

        def terrain_surface(x, y):
            alt = []
            for north in y:
                alt_y = []
                for east in x:
                    alt_y.append(grid_map.atPosition("elevation", (east, north)))
                alt.append(alt_y)
            return alt

        z_grid = np.array(terrain_surface(x, y))
        ax.contour(x_grid, y_grid, z_grid, levels=10, alpha=0.3)
        ax.plot_wireframe(
            x_grid,
            y_grid,
            z_grid,
            linewidth=1,
            linestyle="solid",
            alpha=0.3,
            color="grey",
        )
        # ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.3, color="grey")
        # aspect ratio is 1:1:1 in data space
        ax.set_box_aspect((np.ptp(x_grid), np.ptp(y_grid), 5 * np.ptp(z_grid)))

    # setup plot
    ax = plt.figure().add_subplot(projection="3d")
    ax.set_xlim(-5000.0, 5000.0)
    ax.set_ylim(-5000.0, 5000.0)
    ax.set_zlim(0.0, 1500.0)
    # ax.set_zlim(1000.0, 2500.0)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("Terrain Planner (source: SRTM1)")

    # start circle
    plot_circle(ax, start_pos, loiter_radius, "start")

    # goal circle
    plot_circle(ax, goal_pos, loiter_radius, "goal")

    # path
    if path is not None:
        plot_path(ax, path)

    # states
    if states is not None:
        plot_states(ax, states)

    # map
    if grid_map is not None:
        plot_terrain(ax, grid_map)
        length = grid_map.getLength()
        ax.set_xlim(-0.5 * length[0], 0.5 * length[0])
        ax.set_ylim(-0.5 * length[1], 0.5 * length[1])

    plt.show()


def main():
    # test_terrain_ompl_rrt()
    test_terrain_ompl_rrt_solution_path_to_path()


if __name__ == "__main__":
    main()
