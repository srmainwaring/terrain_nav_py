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

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace
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

    # Davos
    start_lat = 46.8141348
    start_lon = 9.8488310
    goal_lat = 46.8201124
    goal_lon = 9.8260916
    grid_length_factor = 2.0

    # Caswell
    # start_lat = 51.568510858990884
    # start_lon = -4.033516028895548
    # goal_lat = 51.56337235695456
    # goal_lon = -4.033516028895548

    # Langland
    # goal_lat = 51.566326390180855
    # goal_lon = -4.010968419937816

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
    max_altitude = 100.0
    min_altitude = 40.0
    time_budget = 10.0
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

    # NOTE: this does not work because of unmatched signature...
    # use a deterministic sampler
    # space = problem.getStateSpace()
    # def allocDeterministicStateSampler(space):
    #     return ob.DeterministicStateSampler(space)
    # space.setStateSamplerAllocator(allocDeterministicStateSampler)

    candidate_path = Path()
    planner_mgr.Solve1(time_budget=time_budget, path=candidate_path)

    # check states are all above the min_altitude
    solution_path = planner_mgr._problem_setup.getSolutionPath()
    states = solution_path.getStates()

    # take a copy of the states, as the interpolation (below) will modify them.
    states_copy = []
    for state in states:
        states_copy.append(state)
        x = state[0]
        y = state[1]
        z = state[2]
        yaw = state.yaw()

        layer = "elevation"
        elevation = grid_map.atPosition(layer, (x, y, z))
        log.debug(
            f"[{layer}]: "
            f"[{x:.2f}, {y:.2f}, {z:.2f}]; "
            f"ter_alt: {elevation:.2f}, agl_alt: {(z - elevation):.2f}"
        )

    # get the Dubins curve segments
    # NOTE: requires: https://github.com/ompl/ompl/pull/1261
    segments = []
    state_count = solution_path.getStateCount()
    for i in range(state_count - 1):
        from_state = solution_path.getState(i)
        to_state = solution_path.getState(i + 1)
        space = planner_mgr._problem_setup.getStateSpace()
        path_type = space.getPath(from_state, to_state)

        # the Dubins path connecting from_state and to_state
        db_path = path_type.path_

        def to_string(category):
            """Utility to convert enum PathCategory to a string"""
            if category == ob.OwenStateSpace.HIGH_ALTITUDE:
                return "HIGH_ALTITUDE"
            elif category == ob.OwenStateSpace.LOW_ALTITUDE:
                return "LOW_ALTITUDE"
            elif category == ob.OwenStateSpace.MEDIUM_ALTITUDE:
                return "MEDIUM_ALTITUDE"
            else:
                return "UNKNOWN"

        print(
            f"from_state: {from_state[0]:.1f}, {from_state[1]:.1f}, "
            f"{from_state[2]:.1f}, {from_state.yaw():.1f}"
        )
        print(
            f"to_state:   {to_state[0]:.1f}, {to_state[1]:.1f}, "
            f"{to_state[2]:.1f}, {to_state.yaw():.1f}"
        )
        print(f"path_type.category: {to_string(path_type.category())}")
        print(f"path_type.length: {path_type.length():.4f}")
        print(f"path_type.turn_radius: {path_type.turnRadius_:.4f}")
        print(f"path_type.deltaZ: {path_type.deltaZ_:.4f}")
        print(f"path_type.phi: {path_type.phi_:.2f}")
        print(f"path_type.numTurns: {path_type.numTurns_}")
        print(f"path_type.path: {path_type.path_}")
        print(
            f"path_type.path.type: {path_type.path_.type_[0]}, "
            f"{path_type.path_.type_[1]}. {path_type.path_.type_[2]}"
        )
        print(
            f"path_type.path.length: {path_type.path_.length_[0]:.4f}, "
            f"{path_type.path_.length_[1]:.4f}, {path_type.path_.length_[2]:.4f}"
        )
        print(f"path_type.path.reverse: {path_type.path_.reverse_}")

        # skip case where start == end to avoid div zero below
        if path_type.length() == 0.0:
            continue

        # TODO: need the max climb rate from the state space
        def calculateSegmentStarts(
            from_state, path_type, rho, gammaMax
        ) -> DubinsAirplaneStateSpace.SegmentStarts:
            # calculate segments of each Dubins curve
            segmentStarts = DubinsAirplaneStateSpace.SegmentStarts()

            dubins_path = path_type.path_
            dubins_len = (
                dubins_path.length_[0] + dubins_path.length_[1] + dubins_path.length_[2]
            )
            dubins_type = dubins_path.type_

            category = path_type.category()
            ds = path_type.length()
            dz = path_type.deltaZ_
            num_turns = path_type.numTurns_
            radius = path_type.turnRadius_
            phi = path_type.phi_
            hlen = math.sqrt(ds * ds - dz * dz) / radius
            print(f"hlen: {hlen}")
            hlen = dubins_len + 2.0 * math.pi * num_turns + phi
            print(f"hlen: {hlen}")

            interpol_tanGamma = (dz / radius) / hlen
            interpol_seg = hlen

            # OwenStateSpace.PathType may have the following structure
            # LOW_ALTITUDE
            #   - Dubins path
            #   - rho = turn radius
            #
            # MEDIUM_ALTITUDE
            #   - Initial turn of phi followed by a Dubins path
            #   - rho = turn radius
            #
            # HIGH_ALTITUDE
            #   - Initial spiral of k-turns followed by a Dubins path
            #
            def turn_left(phi_start, ds, tan_gamma):
                delta_phi = ds
                tmp = 2 * math.sin(0.5 * delta_phi)
                dx = tmp * math.cos(phi_start + 0.5 * delta_phi)
                dy = tmp * math.sin(phi_start + 0.5 * delta_phi)
                dz = delta_phi * tan_gamma
                yaw = phi_start + delta_phi
                return (dx, dy, dz, yaw)

            def turn_right(phi_start, ds, tan_gamma):
                delta_phi = ds
                tmp = 2 * math.sin(0.5 * delta_phi)
                dx = tmp * math.cos(phi_start - 0.5 * delta_phi)
                dy = tmp * math.sin(phi_start - 0.5 * delta_phi)
                dz = delta_phi * tan_gamma
                yaw = phi_start - delta_phi
                return (dx, dy, dz, yaw)

            def straight(phi_start, ds, tan_gamma):
                delta_phi = ds
                dx = ds * math.cos(phi_start)
                dy = ds * math.sin(phi_start)
                dz = ds * tan_gamma
                yaw = phi_start
                return (dx, dy, dz, yaw)

            # Dubins segments
            # tmp_state is used to enforce yaw bounds
            # interpol_state is for path with turns of unit radius.
            tmp_state = ob.State(space)
            interpol_state = ob.State(space)
            interpol_state[0] = 0.0
            interpol_state[1] = 0.0
            interpol_state[2] = 0.0
            interpol_state().setYaw(from_state.yaw())

            interpol_iter_offset = 0
            if category == ob.OwenStateSpace.LOW_ALTITUDE:
                # only Dubins segments
                pass
            elif category == ob.OwenStateSpace.MEDIUM_ALTITUDE:
                # include an initial turn
                interpol_iter_offset = 1

                interpol_v = min(interpol_seg, phi)
                interpol_seg -= interpol_v
                interpol_phiStart = interpol_state().yaw()

                segmentStarts.segmentStarts[0].x = (
                    interpol_state[0] * rho + from_state[0]
                )
                segmentStarts.segmentStarts[0].y = (
                    interpol_state[1] * rho + from_state[1]
                )
                segmentStarts.segmentStarts[0].z = (
                    interpol_state[2] * rho + from_state[2]
                )

                # enforce bounds using temporary state
                tmp_state[0] = 0.0
                tmp_state[1] = 0.0
                tmp_state[2] = 0.0
                tmp_state().setYaw(interpol_state().yaw())
                space.enforceBounds(tmp_state())
                interpol_state().setYaw(tmp_state().yaw())
                segmentStarts.segmentStarts[0].yaw = interpol_state().yaw()

                if phi > 0.0:
                    (dx, dy, dz, yaw) = turn_left(
                        interpol_phiStart, interpol_v, interpol_tanGamma
                    )
                elif phi < 0.0:
                    (dx, dy, dz, yaw) = turn_right(
                        interpol_phiStart, interpol_v, interpol_tanGamma
                    )
                interpol_state[0] += dx
                interpol_state[1] += dy
                interpol_state[2] += dz
                interpol_state().setYaw(yaw)

            elif category == ob.OwenStateSpace.HIGH_ALTITUDE:
                # include a spiral
                pass

            for interpol_iter in range(3):
                if interpol_seg <= 0.0:
                    break

                # NOTE: DubinsPath length is scaled by turn radius,
                interpol_v = min(interpol_seg, path_type.path_.length_[interpol_iter])
                interpol_seg -= interpol_v
                interpol_phiStart = interpol_state().yaw()

                segmentStarts.segmentStarts[interpol_iter + interpol_iter_offset].x = (
                    interpol_state[0] * rho + from_state[0]
                )
                segmentStarts.segmentStarts[interpol_iter + interpol_iter_offset].y = (
                    interpol_state[1] * rho + from_state[1]
                )
                segmentStarts.segmentStarts[interpol_iter + interpol_iter_offset].z = (
                    interpol_state[2] * rho + from_state[2]
                )

                # TODO: cannot access internal state as compound state?
                # if isinstance(interpol_state, ob.State):
                #     abstract_state = interpol_state
                #     internal_state = interpol_state()
                # elif isinstance(interpol_state, ob.CompoundInternalState):
                #     internal_state = interpol_state
                # so2_space = space.getSubspace(1)
                # so2_state = internal_state[1]
                # so2_space.enforceBounds(so2_state)

                # enforce bounds using temporary state
                tmp_state[0] = 0.0
                tmp_state[1] = 0.0
                tmp_state[2] = 0.0
                tmp_state().setYaw(interpol_state().yaw())
                space.enforceBounds(tmp_state())
                interpol_state().setYaw(tmp_state().yaw())
                segmentStarts.segmentStarts[interpol_iter + interpol_iter_offset].yaw = interpol_state().yaw()

                # calculate change in position and yaw
                segment_type = path_type.path_.type_[interpol_iter]
                if segment_type == ob.DubinsStateSpace.DUBINS_LEFT:
                    (dx, dy, dz, yaw) = turn_left(
                        interpol_phiStart, interpol_v, interpol_tanGamma
                    )
                elif segment_type == ob.DubinsStateSpace.DUBINS_RIGHT:
                    (dx, dy, dz, yaw) = turn_right(
                        interpol_phiStart, interpol_v, interpol_tanGamma
                    )
                elif segment_type == ob.DubinsStateSpace.DUBINS_STRAIGHT:
                    (dx, dy, dz, yaw) = straight(
                        interpol_phiStart, interpol_v, interpol_tanGamma
                    )
                else:
                    raise ValueError("Invalid segment type")
                interpol_state[0] += dx
                interpol_state[1] += dy
                interpol_state[2] += dz
                interpol_state().setYaw(yaw)

            return segmentStarts

        segmentStarts = calculateSegmentStarts(
            from_state,
            path_type=path_type,
            rho=turning_radius,
            gammaMax=climb_angle_rad,
        )
        segments.append(segmentStarts)

    # NOTE: approximate path construction for visualisation, as
    #       OwenStateSpace is missing additional methods present in
    #       DubinsAirplaneStateSpace.
    candidate_path = Path()

    path = planner_mgr._problem_setup.getSolutionPath()
    print(f"solution path len: {len(path.getStates())}")
    path.interpolate(1000)
    print(f"interpolated path len: {len(path.getStates())}")
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
            segments=segments,
        )


def main():
    test_owen_state_space_model()


if __name__ == "__main__":
    main()
