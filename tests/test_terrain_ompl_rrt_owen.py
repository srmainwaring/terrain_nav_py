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

import copy
import logging
import math

from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou

from ompl.base import OwenStateSpace

from pymavlink import quaternion
from pymavlink.rotmat import Matrix3
from pymavlink.rotmat import Vector3

from MAVProxy.modules.lib import mp_util

from terrain_nav_py import add_stderr_logger

# from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace
from terrain_nav_py.grid_map import GridMapSRTM
from terrain_nav_py.path import Path
from terrain_nav_py.path_segment import PathSegment
from terrain_nav_py.path_segment import State
from terrain_nav_py.terrain_map import TerrainMap
from terrain_nav_py.terrain_ompl_rrt_owen import TerrainOmplRrtOwen

from test_terrain_ompl_rrt import plot_path

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


def test_owen_state_space_model():
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

    # Davos
    start_lat = 46.8141348
    start_lon = 9.8488310
    goal_lat = 46.8201124
    goal_lon = 9.8260916
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
    loiter_radius = 90.0
    loiter_alt = 60.0
    turning_radius = 90.0
    climb_angle_rad = 0.15
    max_altitude = 120.0
    min_altitude = 50.0
    time_budget = 5.0
    resolution_m = 200.0

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
    da_space = OwenStateSpace(turningRadius=turning_radius, maxPitch=climb_angle_rad)
    planner_mgr = TerrainOmplRrtOwen(da_space)
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
    is_start_valid = TerrainOmplRrtOwen.validatePosition(
        grid_map, start_pos, loiter_radius
    )
    if not is_start_valid:
        log.debug(f"Invalid start position: {start_pos}")
    else:
        log.debug(f"Accept start position: {start_pos}")
    is_goal_valid = TerrainOmplRrtOwen.validatePosition(
        grid_map, start_pos, loiter_radius
    )
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

    # check states are all above the min_altitude
    solution_path = planner_mgr._problem_setup.getSolutionPath()
    states = solution_path.getStates()
    states_copy = []
    for state in states:
        states_copy.append(state)
        # da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        da_state = state
        # (x, y, z, yaw) = da_state.getXYZYaw()
        x = da_state[0]
        y = da_state[1]
        z = da_state[2]
        yaw = da_state.yaw()

        layer = "elevation"
        elevation = grid_map.atPosition(layer, (x, y, z))
        log.debug(
            f"[{layer}]: "
            f"[{x:.2f}, {y:.2f}, {z:.2f}]; "
            f"ter_alt: {elevation:.2f}, agl_alt: {(z - elevation):.2f}"
        )

    # NOTE: approximaste path construction for visualisation, as
    #       OwenStateSpace is missing additional methods present in
    #       DubinsAirplaneStateSpace.
    path = planner_mgr._problem_setup.getSolutionPath()
    print(f"path len: {len(path.getStates())}")
    path.interpolate(1000)
    print(f"path len: {len(path.getStates())}")
    candidate_path = Path()
    path_segment = PathSegment()
    for state in path.getStates():
        x = state[0]
        y = state[1]
        z = state[2]
        yaw = state.yaw()
        m_att = Matrix3()
        m_att.from_euler(0.0, 0.0, yaw)
        path_state = State()
        path_state.position = Vector3(x, y, z)
        path_state.velocity = Vector3(0.0, 0.0, 0.0)
        path_state.attitude = quaternion.Quaternion(m_att)
        path_segment.append_state(path_state)
    candidate_path.append_segment(path_segment)

    # only display plots if run as script
    if __name__ == "__main__":
        plot_path(
            start_pos,
            goal_pos,
            loiter_radius,
            candidate_path,
            states=states_copy,
            grid_map=grid_map,
        )


def main():
    test_owen_state_space_model()


if __name__ == "__main__":
    main()
