"""
Terrain planner OMPL RRT
"""

# Copyright (c) 2025 Rhys Mainwaring. All rights reserved.

# BSD-3-Clause and copyright notice from original C++ version.
#
# Copyright (c) 2021-2023 Jaeyoung Lim, Autonomous Systems Lab,
# ETH Zürich. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import math
import sys

import numpy as np

from ompl import base as ob
from ompl import geometric as og

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace
from terrain_nav_py.dubins_path import DubinsPath

from terrain_nav_py import path_segment
from terrain_nav_py.path_segment import wrap_pi
from terrain_nav_py.path_segment import wrap_2pi

from terrain_nav_py.path import Path
from terrain_nav_py.path import PathSegment

from terrain_nav_py.ompl_setup import OmplSetup

from terrain_nav_py.terrain_ompl import GridMap
from terrain_nav_py.terrain_ompl import TerrainStateSampler


# Mock interface for TerrainMap
class TerrainMap:
    def __init__(self):
        self._grid_map = GridMap()

    def getGridMap(self) -> GridMap:
        return self._grid_map

    def setGridMap(self, map: GridMap) -> None:
        self._grid_map = map


class TerrainOmplRrt:
    """
    Setup and run the planning problem.
    """

    def __init__(self, space: ob.StateSpace):
        # TODO: test
        # TODO: locate definition of TerrainMap
        # self._minimum_turning_radius: float = 66.67
        self._problem_setup: OmplSetup = OmplSetup(space)
        self._map: TerrainMap = None
        self._min_altitude: float = 50.0
        self._max_altitude: float = 120.0
        self._planner_data: ob.PlannerData = None
        self._goal_states: ob.GoalStates = None
        self._lower_bound = (0.0, 0.0, 0.0)
        self._upper_bound = (0.0, 0.0, 0.0)
        self._solve_duration: float = 0.0
        self._check_max_altitude: bool = True

    def configureProblem(self) -> None:
        """
        Configure OMPL problem descriptions
        """
        # TODO: test
        self._problem_setup.clear()
        self._problem_setup.clearStartStates()

        self._problem_setup.setDefaultPlanner()
        self._problem_setup.setDefaultObjective()

        grid_map = self._map.getGridMap()
        self._problem_setup.setTerrainCollisionChecking(
            grid_map, self._check_max_altitude
        )

        # self._problem_setup.getStateSpace()->setStateSamplerAllocator(
        #      std::bind(&TerrainOmplRrt::allocTerrainStateSampler, this, std::placeholders::_1));
        # self._problem_setup.getStateSpace().allocStateSampler()

        bounds = ob.RealVectorBounds(3)
        bounds.setLow(0, self._lower_bound[0])
        bounds.setLow(1, self._lower_bound[1])
        bounds.setLow(2, self._lower_bound[2])

        bounds.setHigh(0, self._upper_bound[0])
        bounds.setHigh(1, self._upper_bound[1])
        bounds.setHigh(2, self._upper_bound[2])

        # define start and goal positions.
        # da_space = self._problem_setup.getGeometricComponentStateSpace()
        da_space = self._problem_setup.getStateSpace()
        da_space.setBounds(bounds)

        self._problem_setup.setStateValidityCheckingResolution(0.001)

        si = self._problem_setup.getSpaceInformation()
        self._planner_data = ob.PlannerData(si)

    def setupProblem1(
        self, start_pos: tuple[float, float, float], goal: tuple[float, float, float]
    ) -> None:
        """
        Setup problem with center position of start and goal loiter circles

        :param start_pos: center of the start loiter position
        :param goal: center of the goal loiter position
        """
        # TODO: test
        da_space = self._problem_setup.getStateSpace()
        start_loiter_radius = da_space.getMinTurningRadius()
        self.setupProblem2(start_pos, goal, start_loiter_radius)

    def setupProblem2(
        self,
        start_pos: tuple[float, float, float],
        goal: tuple[float, float, float],
        start_loiter_radius: float,
    ) -> None:
        """
        Setup problem with center position of start and goal loiter circle with specific radius

        :param start_pos:
        :param goal:
        :param start_loiter_radius:
        """
        # TODO: test
        self.configureProblem()

        da_space = self._problem_setup.getStateSpace()
        radius = da_space.getMinTurningRadius()

        delta_theta = 0.1
        theta_samples = np.arange(-math.pi, math.pi, delta_theta * 2 * math.pi)

        # for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
        for theta in theta_samples:
            # TODO: check whether to use ob.State or do we need to define a separate scoped one in Python?

            # ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> start_ompl(
            #     self._problem_setup.getSpaceInformation());
            start_state = ob.State(da_space)
            start_ompl = DubinsAirplaneStateSpace.DubinsAirplaneState(start_state)

            start_ompl.setX(
                start_pos[0] + math.fabs(start_loiter_radius) * math.cos(theta)
            )
            start_ompl.setY(
                start_pos[1] + math.fabs(start_loiter_radius) * math.sin(theta)
            )
            start_ompl.setZ(start_pos[2])
            start_yaw = (
                theta - 2.0 * math.pi
                if start_loiter_radius > 0.0
                else theta + 2.0 * math.pi
            )
            start_yaw = wrap_pi(start_yaw)
            start_ompl.setYaw(start_yaw)
            # TODO scoped vs abstract state
            # self._problem_setup.addStartState(start_ompl)
            self._problem_setup.addStartState(start_state)

        self._goal_states = ob.GoalStates(self._problem_setup.getSpaceInformation())

        # for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
        for theta in theta_samples:
            # TODO: check whether to use ob.State or do we need to define a separate scoped one in Python?

            # ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> goal_ompl(
            #     self._problem_setup.getSpaceInformation());
            goal_state = ob.State(da_space)
            goal_ompl = DubinsAirplaneStateSpace.DubinsAirplaneState(goal_state)

            goal_ompl.setX(goal[0] + radius * math.cos(theta))
            goal_ompl.setY(goal[1] + radius * math.sin(theta))
            goal_ompl.setZ(goal[2])
            goal_yaw = theta + 2.0 * math.pi
            goal_yaw = wrap_pi(goal_yaw)
            goal_ompl.setYaw(goal_yaw)
            # TODO scoped vs abstract state
            # self._goal_states.addState(goal_ompl)
            self._goal_states.addState(goal_state)
            goal_yaw = theta - 2.0 * math.pi
            goal_yaw = wrap_pi(goal_yaw)
            goal_ompl.setYaw(goal_yaw)
            # Add additional state for bidirectional tangents
            # TODO scoped vs abstract state
            # self._goal_states.addState(goal_ompl)
            self._goal_states.addState(goal_state)

        self._problem_setup.setGoal(self._goal_states)
        self._problem_setup.setup()

        planner_ptr = self._problem_setup.getPlanner()
        # TODO: check cast of planner_ptr to og::RRTstar
        print(f"Planner Range: {planner_ptr.getRange()}")

    def setupProblem3(
        self,
        start_pos: tuple[float, float, float],
        start_vel: tuple[float, float, float],
        goal: tuple[float, float, float],
        goal_radius: float = -1.0,
    ) -> None:
        """
        Setup problem with position, velocity of the start and center of the goal loiter circle

        :param start_pos: position of the start state
        :param start_vel: velocity of the start state
        :param goal: center of the goal loiter position
        """
        # TODO: test
        self.configureProblem()

        da_space = self._problem_setup.getStateSpace()
        radius = da_space.getMinTurningRadius()

        radius = goal_radius
        if goal_radius < 0.0:
            radius = da_space.getMinTurningRadius()

        delta_theta = 0.1
        theta_samples = np.arange(-math.pi, math.pi, delta_theta * 2 * math.pi)

        # TODO: check (see above)
        # ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> start_ompl(
        #     self._problem_setup.getSpaceInformation())
        start_state = ob.State(da_space)
        start_ompl = DubinsAirplaneStateSpace.DubinsAirplaneState(start_state)

        start_ompl.setX(start_pos(0))
        start_ompl.setY(start_pos(1))
        start_ompl.setZ(start_pos(2))
        start_yaw = math.atan2(start_vel[1], start_vel[0])
        start_ompl.setYaw(start_yaw)
        self._problem_setup.clearStartStates()  # Clear previous goal states
        # TODO scoped vs abstract state
        # self._problem_setup.addStartState(start_ompl)
        self._problem_setup.addStartState(start_state)

        self._goal_states = ob.GoalStates(self._problem_setup.getSpaceInformation())

        # for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
        for theta in theta_samples:
            # ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> goal_ompl(
            #     self._problem_setup.getSpaceInformation());
            goal_state = ob.State(da_space)
            goal_ompl = DubinsAirplaneStateSpace.DubinsAirplaneState(goal_state)

            goal_ompl.setX(goal[0] + radius * math.cos(theta))
            goal_ompl.setY(goal[1] + radius * math.sin(theta))
            goal_ompl.setZ(goal[2])
            goal_yaw = theta + 2.0 * math.pi
            goal_yaw = wrap_pi(goal_yaw)
            goal_ompl.setYaw(goal_yaw)
            # TODO scoped vs abstract state
            # self._goal_states.addState(goal_ompl)
            self._goal_states.addState(goal_state)
            goal_yaw = theta - 2.0 * math.pi
            goal_yaw = wrap_pi(goal_yaw)
            goal_ompl.setYaw(goal_yaw)
            # Add additional state for bidirectional tangents
            # TODO scoped vs abstract state
            # self._goal_states.addState(goal_ompl)
            self._goal_states.addState(goal_state)

        self._problem_setup.setGoal(self._goal_states)
        self._problem_setup.setup()

    def setupProblem4(
        self,
        start_pos: tuple[float, float, float],
        start_vel: tuple[float, float, float],
        goal_positions: list[tuple[float, float, float]],
    ) -> None:
        """
        Setup problem with position, velocity of the start and goal state

        :param start_pos: position of the start state
        :param start_vel: velocity of the start state
        :param goal: position of the goal state
        :param goal_vel: velocity of the goal state
        """
        # TODO: test
        if goal_positions.empty():
            # TODO: should raise exception?
            print("Failed to configure problem: Goal position list empty")
            return

        self.configureProblem()

        da_space = self._problem_setup.getStateSpace()
        radius = da_space.getMinTurningRadius()

        radius = da_space.getMinTurningRadius()

        delta_theta = 0.1
        theta_samples = np.arange(-math.pi, math.pi, delta_theta * 2 * math.pi)

        # TODO: check (see above)
        # ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> start_ompl(
        #     self._problem_setup.getSpaceInformation());
        start_state = ob.State(da_space)
        start_ompl = DubinsAirplaneStateSpace.DubinsAirplaneState(start_state)

        start_ompl.setX(start_pos(0))
        start_ompl.setY(start_pos(1))
        start_ompl.setZ(start_pos(2))
        start_yaw = math.atan2(start_vel[1], start_vel[0])
        start_ompl.setYaw(start_yaw)
        self._problem_setup.clearStartStates()  # Clear previous goal states
        # TODO scoped vs abstract state
        # self._problem_setup.addStartState(start_ompl)
        self._problem_setup.addStartState(start_state)

        self._goal_states = ob.GoalStates(self._problem_setup.getSpaceInformation())

        for goal in goal_positions:
            for theta in theta_samples:
                goal_state = ob.State(da_space)
                goal_ompl = DubinsAirplaneStateSpace.DubinsAirplaneState(goal_state)

                goal_ompl.setX(goal[0] + radius * math.cos(theta))
                goal_ompl.setY(goal[1] + radius * math.sin(theta))
                goal_ompl.setZ(goal[2])
                goal_yaw = theta + 2.0 * math.pi
                goal_yaw = wrap_pi(goal_yaw)
                goal_ompl.setYaw(goal_yaw)
                # TODO scoped vs abstract state
                # self._goal_states.addState(goal_ompl)
                self._goal_states.addState(goal_state)
                goal_yaw = theta - 2.0 * math.pi
                goal_yaw = wrap_pi(goal_yaw)
                goal_ompl.setYaw(goal_yaw)
                # Add additional state for bidirectional tangents
                # TODO scoped vs abstract state
                # self._goal_states.addState(goal_ompl)
                self._goal_states.addState(goal_state)

        self._problem_setup.setGoal(self._goal_states)
        self._problem_setup.setup()

    def setupProblem5(
        self,
        start_pos: tuple[float, float, float],
        start_vel: tuple[float, float, float],
        goal: tuple[float, float, float],
        goal_vel: tuple[float, float, float],
    ) -> None:
        """
        Setup problem with position, velocity of the start and goal state

        :param start_pos: position of the start state
        :param start_vel: velocity of the start state
        :param goal: position of the goal state
        :param goal_vel: velocity of the goal state
        """
        # TODO: test
        self.configureProblem()

        da_space = self._problem_setup.getStateSpace()

        # ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> start_ompl(
        #     self._problem_setup.getSpaceInformation())
        start_state = ob.State(da_space)
        start_ompl = DubinsAirplaneStateSpace.DubinsAirplaneState(start_state)

        # ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> goal_ompl(
        #     self._problem_setup.getSpaceInformation());
        goal_state = ob.State(da_space)
        goal_ompl = DubinsAirplaneStateSpace.DubinsAirplaneState(goal_state)

        start_ompl.setX(start_pos[0])
        start_ompl.setY(start_pos[1])
        start_ompl.setZ(start_pos[2])
        start_yaw = math.atan2(start_vel[1], start_vel[0])
        start_ompl.setYaw(start_yaw)

        goal_ompl.setX(goal[0])
        goal_ompl.setY(goal[1])
        goal_ompl.setZ(goal[2])
        goal_yaw = math.atan2(goal_vel[1], goal_vel[0])
        goal_ompl.setYaw(goal_yaw)

        self._problem_setup.setStartAndGoalStates(start_ompl, goal_ompl)
        self._problem_setup.setup()

    def setBounds(
        self,
        lower_bound: tuple[float, float, float],
        upper_bound: tuple[float, float, float],
    ) -> None:
        """
        Set the Bounds of the statespace for the planner

        :param lower_bound: lower bo
        :param upper_bound:
        """
        # TODO: test
        self._lower_bound_ = lower_bound
        self._upper_bound_ = upper_bound

    def setBoundsFromMap(self, map: GridMap) -> None:
        """
        Set the Bounds of the statespace for the planner using the map

        :param map:
        """
        # TODO: test
        # NOTE: GridMap must support an iterator interface
        map_pos = map.getPosition()

        # Iterate through map to get elevation bounds
        min_elevation = sys.float_info.max
        max_elevation = sys.float_info.min
        for map_index in map:
            minimum_elevation_limit = map.at("distance_surface", map_index)
            if minimum_elevation_limit < min_elevation:
                min_elevation = minimum_elevation_limit

            maximum_elevation_limit = map.at("max_elevation", map_index)
            if maximum_elevation_limit > max_elevation:
                max_elevation = maximum_elevation_limit

        map_width = map.getLength()
        map_width_x = map_width[0]
        map_width_y = map_width[1]
        roi_ratio = 0.5
        lower_bounds = (
            map_pos[0] - roi_ratio * map_width_x,
            map_pos[1] - roi_ratio * map_width_y,
            min_elevation,
        )
        upper_bounds = (
            map_pos[0] + roi_ratio * map_width_x,
            map_pos[1] + roi_ratio * map_width_y,
            max_elevation,
        )
        print(f"[TerrainOmplRrt] Upper bounds: {upper_bounds}")
        print(f"[TerrainOmplRrt] Lower bounds: {lower_bounds}")
        self.setBounds(lower_bounds, upper_bounds)

    def setMap(self, map: TerrainMap) -> None:
        """
        Set the Map

        :param map:
        """
        # TODO: test
        self._map = map

    def setMaxAltitudeCollisionChecks(self, check_max_altitude: bool) -> None:
        """
        Set the Max Altitude Collision Check

        :param check_max_altitude: If true, enables the maximum altitude collision checks
        """
        # TODO: test
        self._check_max_altitude = check_max_altitude

    def Solve1(self, time_budget: float, path: Path) -> bool:
        """
        Solve the planning problem for a given time budget, and return a
        TerrainSegments object if an exact solution is found

        :param time_budget [s] time the planner should use for planning
        :param path:
        :return true: Found exact solution
        :return false: Did not find an exact solution
        """
        # TODO: test
        if self._problem_setup.solve(time_budget):
            # self._problem_setup.getSolutionPath().print(std::cout)
            # self._problem_setup.simplifySolution()
            # self._problem_setup.getSolutionPath().print(std::cout)
            self._problem_setup.getPlannerData(self._planner_data)
            self._solve_duration = self._problem_setup.getLastPlanComputationTime()

        else:
            print("Solution Not found")

        if self._problem_setup.haveExactSolutionPath():
            print("Found Exact solution!")
            self.solutionPathToPath(self._problem_setup.getSolutionPath(), path)
            return True

        return False

    def Solve2(self, time_budget: float, path: list[tuple[float, float, float]]) -> bool:
        # TODO: test
        if self._problem_setup.solve(time_budget):
            print("Found solution:")
            # self._problem_setup.getSolutionPath().print(std::cout);
            # self._problem_setup.simplifySolution();
            # self._problem_setup.getSolutionPath().print(std::cout);
            self._problem_setup.getPlannerData(self._planner_data)
            self._solve_duration = self._problem_setup.getLastPlanComputationTime()

        else:
            print("Solution Not found")

        if self._problem_setup.haveExactSolutionPath():
            self.solutionPathToTrajectoryPoints(
                self._problem_setup.getSolutionPath(), path
            )
            return True

        return False

    def getSegmentCurvature(
        self, problem_setup: OmplSetup, dubins_path: DubinsPath, start_idx: int
    ) -> float:
        # TODO: test
        # TODO: difference between getGeometricComponentStateSpace and getStateSpace
        # NOTE: they are equivalent in Python (one is the derived type in C++) 
        # da_space = problem_setup.getGeometricComponentStateSpace()
        da_space = problem_setup.getStateSpace()

        segment_curvature = 0.0
        maximum_curvature = 1.0 / da_space.getMinTurningRadius()

        # TODO: see NOTE above
        # da_space = problem_setup.getStateSpace()
        db_idx = da_space.convert_idx(start_idx)
        dubins_path_type = dubins_path.getType()[db_idx]
        # switch (
        #     dubins_path
        #         .getType()[problem_setup->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->convert_idx(
        #             start_idx)]) {
        if dubins_path_type == DubinsPath.DUBINS_LEFT:
            segment_curvature = maximum_curvature
        elif dubins_path_type == DubinsPath.DUBINS_RIGHT:
            segment_curvature = -maximum_curvature
        elif dubins_path_type == DubinsPath.DUBINS_STRAIGHT:
            segment_curvature = 0.0

        return segment_curvature

    def solutionPathToPath(
        self,
        path: og.PathGeometric,
        trajectory_segments: Path,
        resolution: float = 0.05,
    ) -> None:
        # TODO: test
        trajectory_segments.segments.clear()

        state_vector = path.getStates()

        # for (size_t idx = 0; idx < state_vector.size() - 1; idx++) {
        for idx in range(len(state_vector) - 1):
            from_state = state_vector[idx]  # Start of the segment
            to_state = state_vector[idx + 1]  # End of the segment
            da_space = self._problem_setup.getStateSpace()

            dubins_path = DubinsPath()
            da_space.dubins(from_state, to_state, dubins_path)

            segmentStarts = DubinsAirplaneStateSpace.SegmentStarts()
            da_space.calculateSegments(from_state, to_state, dubins_path, segmentStarts)

            # segment_start_state: ob.State = self._problem_setup.getStateSpace()->allocState();
            # segment_end_state: ob.State = self._problem_setup.getStateSpace()->allocState();
            segment_start_state = ob.State(da_space)
            segment_end_state = ob.State(da_space)

            total_length = dubins_path.length_2d()
            dt = resolution / total_length
            progress = 0.0
            # for (size_t start_idx = 0; start_idx < segmentStarts.segmentStarts.size(); start_idx++) {
            for start_idx in range(len(segmentStarts.segmentStarts)):
                if dubins_path.getSegmentLength(start_idx) > 0.0:
                    segment_progress = (
                        dubins_path.getSegmentLength(start_idx) / total_length
                    )
                    # Read segment start and end statess
                    TerrainOmplRrt.segmentStart2omplState(
                        segmentStarts.segmentStarts[start_idx], segment_start_state
                    )
                    if (start_idx + 1) > (len(segmentStarts.segmentStarts) - 1):
                        segment_end_state = to_state
                    elif (start_idx + 1) > (
                        len(segmentStarts.segmentStarts) - 2
                    ) and dubins_path.getSegmentLength(start_idx + 1) == 0.0:
                        segment_end_state = to_state
                    else:
                        TerrainOmplRrt.segmentStart2omplState(
                            segmentStarts.segmentStarts[start_idx + 1],
                            segment_end_state,
                        )

                    # Append to trajectory
                    trajectory = PathSegment()
                    trajectory.curvature = self.getSegmentCurvature(
                        self._problem_setup, dubins_path, start_idx
                    )
                    # ompl::base::State* state = self._problem_setup.getStateSpace()->allocState()
                    state = ob.State(da_space)
                    trajectory.flightpath_angle = dubins_path.getGamma()
                    yaw = 0.0
                    track_progress = 0.0
                    t_samples = np.arange(progress, progress + segment_progress, dt)
                    # for (double t = progress; t <= progress + segment_progress; t = t + dt):
                    for t in t_samples:
                        segment_state = path_segment.State()
                        # self._problem_setup.getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->interpolate(
                        #     dubins_path, segmentStarts, t, state)
                        da_space.interpolate(dubins_path, segmentStarts, t, state)

                        position = TerrainOmplRrt.dubinsairplanePosition(state)
                        yaw = TerrainOmplRrt.dubinsairplaneYaw(state)
                        velocity = (math.cos(yaw), math.sin(yaw), 0.0)
                        segment_state.position = position
                        segment_state.velocity = velocity
                        segment_state.attitude = (
                            math.cos(yaw / 2.0),
                            0.0,
                            0.0,
                            math.sin(yaw / 2.0),
                        )
                        trajectory.states.append(segment_state)
                        track_progress = t

                    # Append end state
                    if ((start_idx + 1) > (segmentStarts.segmentStarts.size() - 1)) or (
                        (start_idx + 1) > (segmentStarts.segmentStarts.size() - 2)
                        and dubins_path.getSegmentLength(start_idx + 1) == 0.0
                    ):
                        # Append segment with last state
                        end_state = path_segment.State()
                        end_position = TerrainOmplRrt.dubinsairplanePosition(
                            segment_end_state
                        )
                        end_yaw = TerrainOmplRrt.dubinsairplaneYaw(segment_end_state)
                        end_velocity = (math.cos(end_yaw), math.sin(end_yaw), 0.0)
                        end_state.position = end_position
                        end_state.velocity = end_velocity
                        end_state.attitude = (
                            math.cos(end_yaw / 2.0),
                            0.0,
                            0.0,
                            math.sin(end_yaw / 2.0),
                        )
                        trajectory.states.emplace_back(end_state)

                    progress = track_progress
                    # Do not append trajectory if the segment is too short
                    if len(trajectory.states) > 1:
                        trajectory_segments.segments.append(trajectory)

    def solutionPathToTrajectoryPoints(
        self,
        path: og.PathGeometric,
        trajectory_points: list[tuple[float, float, float]],
    ) -> None:
        # TODO: test
        # trajectory_points.clear()
        trajectory_points = []
        path.interpolate()

        state_vector = path.getStates()

        for state_ptr in state_vector:
            position = TerrainOmplRrt.dubinsairplanePosition(state_ptr)
            trajectory_points.append(position)

        return trajectory_points

    def getPlannerData(self) -> ob.PlannerData:
        # TODO: test
        return self._planner_data

    def getProblemSetup(self) -> OmplSetup:
        # TODO: test
        return self._problem_setup

    def allocTerrainStateSampler(self, space: ob.StateSpace) -> ob.StateSampler:
        # TODO: test
        sampler = TerrainStateSampler(
            space, self._map.getGridMap(), self._max_altitude, self._min_altitude
        )
        return sampler

    def getSolutionPathLength(self) -> tuple[bool, float]:
        # TODO: test
        if self._problem_setup.haveExactSolutionPath():
            path: og.PathGeometric = self._problem_setup.getSolutionPath()
            path.interpolate()
            path_length = path.length()
            return (True, path_length)

        return (False, float("nan"))

    def getSolutionPath(self) -> tuple[bool, list[tuple[float, float, float]]]:
        # TODO: test
        if self._problem_setup.haveExactSolutionPath():
            path = self.solutionPathToTrajectoryPoints(
                self._problem_setup.getSolutionPath()
            )
            return (True, path)

        return (False, None)

    def getSolutionTime(self) -> float:
        # TODO: test
        return self._solve_duration

    @staticmethod
    def dubinsairplanePosition(state_ptr: ob.State) -> tuple[float, float, float]:
        # TODO: test
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state_ptr)
        position = (da_state.getX(), da_state.getY(), da_state.getZ())
        return position

    @staticmethod
    def dubinsairplaneYaw(state_ptr: ob.State) -> float:
        # TODO: test
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state_ptr)
        yaw = da_state.getYaw()
        return yaw

    @staticmethod
    def segmentStart2omplState(
        start: DubinsAirplaneStateSpace.SegmentStarts.Start, state: ob.State
    ) -> None:
        # TODO: test
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        da_state.setXYZYaw(start.x, start.y, start.x, start.yaw)

    def setAltitudeLimits(self, max_altitude: float, min_altitude: float) -> None:
        # TODO: test
        self._max_altitude = max_altitude
        self._min_altitude = min_altitude
