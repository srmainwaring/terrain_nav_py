# BSD 3-Clause License
#
# Copyright (c) 2025, Rhys Mainwaring
#
# Ported to Python from original C++ code in
# https://github.com/ethz-asl/terrain-navigation.git
#
# Copyright (c) 2021-2023 Jaeyoung Lim, Autonomous Systems Lab,
# ETH Zürich. All rights reserved.
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

"""
Terrain planner OMPL RRT
"""

import math
import sys

import numpy as np

from ompl import base as ob
from ompl import geometric as og

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace
from terrain_nav_py.dubins_path import DubinsPath

from terrain_nav_py.grid_map import GridMap

from terrain_nav_py.ompl_setup import OmplSetup

from terrain_nav_py import path_segment
from terrain_nav_py.util import wrap_pi

from terrain_nav_py.path import Path
from terrain_nav_py.path import PathSegment

from terrain_nav_py.terrain_map import TerrainMap

from terrain_nav_py.terrain_ompl import TerrainStateSampler


class TerrainOmplRrt:
    """
    Setup and run the planning problem.
    """

    def __init__(self, space: ob.StateSpace):
        # TODO: test
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
        print(f"[TerrainOmplRrt] configureProblem")

        print(f"[TerrainOmplRrt] clearing previous states")
        self._problem_setup.clear()
        self._problem_setup.clearStartStates()

        print(f"[TerrainOmplRrt] setup default planner and objective")
        self._problem_setup.setDefaultPlanner()
        self._problem_setup.setDefaultObjective()

        # TODO: DEBUG
        # planner = self._problem_setup.getPlanner()
        # print(f"[TerrainOmplRrt] planner: {planner}")
        # objective = self._problem_setup.getOptimizationObjective()
        # print(f"[TerrainOmplRrt] objective: {objective}")
        # TODO: DEBUG

        print(f"[TerrainOmplRrt] setup terrain collition checking")
        grid_map = self._map.getGridMap()
        self._problem_setup.setTerrainCollisionChecking(
            grid_map, self._check_max_altitude
        )

        # NOTE: C++ version is not using the TerrainStateSampler
        # self._problem_setup.getStateSpace()->setStateSamplerAllocator(
        #      std::bind(&TerrainOmplRrt::allocTerrainStateSampler, this, std::placeholders::_1));
        # self._problem_setup.getStateSpace().allocStateSampler()

        print(f"[TerrainOmplRrt] get lower bounds")
        bounds = ob.RealVectorBounds(3)
        bounds.setLow(0, self._lower_bound[0])
        bounds.setLow(1, self._lower_bound[1])
        bounds.setLow(2, self._lower_bound[2])

        print(f"[TerrainOmplRrt] get upper bounds")
        bounds.setHigh(0, self._upper_bound[0])
        bounds.setHigh(1, self._upper_bound[1])
        bounds.setHigh(2, self._upper_bound[2])

        # define start and goal positions.
        da_space = self._problem_setup.getStateSpace()

        print(f"[TerrainOmplRrt] set bounds")
        da_space.setBounds(bounds)

        print(f"[TerrainOmplRrt] setup validity check resolution")
        self._problem_setup.setStateValidityCheckingResolution(0.001)

        print(f"[TerrainOmplRrt] setup planner data ")
        si = self._problem_setup.getSpaceInformation()
        self._planner_data = ob.PlannerData(si)

    # setup using start and goal positions and default start loiter radius
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

    # setup using start and goal positions and start loiter radius
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
        print(f"[TerrainOmplRrt] setupProblem2")
        self.configureProblem()

        da_space = self._problem_setup.getStateSpace()
        print(f"[TerrainOmplRrt] type(da_space): {da_space}")
        radius = da_space.getMinTurningRadius()
        print(f"[TerrainOmplRrt] radius: {radius}")

        num_step = 10
        theta_samples = np.linspace(-math.pi, math.pi, num_step, endpoint=False)

        for theta in theta_samples:

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
                theta - 0.5 * math.pi
                if start_loiter_radius > 0.0
                else theta + 0.5 * math.pi
            )
            start_yaw = wrap_pi(start_yaw)
            start_ompl.setYaw(start_yaw)
            # print(f"[TerrainOmplRrt] adding start state to problem")
            # TODO scoped vs abstract state
            # self._problem_setup.addStartState(start_ompl)
            self._problem_setup.addStartState(start_state)

        self._goal_states = ob.GoalStates(self._problem_setup.getSpaceInformation())
        # print(f"[TerrainOmplRrt] type(goal_states): {type(self._goal_states)}")

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
            goal_yaw = theta + 0.5 * math.pi
            goal_yaw = wrap_pi(goal_yaw)
            goal_ompl.setYaw(goal_yaw)
            # TODO scoped vs abstract state
            # self._goal_states.addState(goal_ompl)
            self._goal_states.addState(goal_state)
            goal_yaw = theta - 0.5 * math.pi
            goal_yaw = wrap_pi(goal_yaw)
            goal_ompl.setYaw(goal_yaw)
            # Add additional state for bidirectional tangents
            # TODO scoped vs abstract state
            # self._goal_states.addState(goal_ompl)
            self._goal_states.addState(goal_state)

        print(f"[TerrainOmplRrt] setting problem goals")
        self._problem_setup.setGoal(self._goal_states)

        # TODO: DEBUG inspect problem definition
        problem_def = self._problem_setup.getProblemDefinition()
        print(f"[TerrainOmplRrt] type(problem_def): {type(problem_def)}")
        # print(f"[TerrainOmplRrt] problem_def:\n{problem_def}")

        # TODO: DEBUG inspect bounds
        da_space = self._problem_setup.getStateSpace()
        re3_space = da_space.getSubspace(0)
        re3_bounds = re3_space.getBounds()
        print(
            f"[TerrainOmplRrt] re3_space.getBounds: "
            f"low: {re3_bounds.low[0], re3_bounds.low[1], re3_bounds.low[2]}"
        )
        print(
            f"[TerrainOmplRrt] re3_space.getBounds: "
            f"high: {re3_bounds.high[0], re3_bounds.high[1], re3_bounds.high[2]}"
        )

        print(f"[TerrainOmplRrt] running problem setup")
        self._problem_setup.setup()

        print(f"[TerrainOmplRrt] get planner from problem")
        planner = self._problem_setup.getPlanner()
        # print(f"[TerrainOmplRrt] type(planner): {type(planner)}")
        print(f"[TerrainOmplRrt] planner range: {planner.getRange()}")

    # setup using start position and velocity and goal position and loiter radius
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

        num_step = 10
        theta_samples = np.linspace(-math.pi, math.pi, num_step, endpoint=False)

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
            goal_yaw = theta + 0.5 * math.pi
            goal_yaw = wrap_pi(goal_yaw)
            goal_ompl.setYaw(goal_yaw)
            # TODO scoped vs abstract state
            # self._goal_states.addState(goal_ompl)
            self._goal_states.addState(goal_state)
            goal_yaw = theta - 0.5 * math.pi
            goal_yaw = wrap_pi(goal_yaw)
            goal_ompl.setYaw(goal_yaw)
            # Add additional state for bidirectional tangents
            # TODO scoped vs abstract state
            # self._goal_states.addState(goal_ompl)
            self._goal_states.addState(goal_state)

        self._problem_setup.setGoal(self._goal_states)
        self._problem_setup.setup()

    # setup using start position and velocity and rally points
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
            print(
                f"[TerrainOmplRrt] Failed to configure problem: Goal position list empty"
            )
            return

        self.configureProblem()

        da_space = self._problem_setup.getStateSpace()
        radius = da_space.getMinTurningRadius()

        radius = da_space.getMinTurningRadius()

        num_step = 10
        theta_samples = np.linspace(-math.pi, math.pi, num_step, endpoint=False)

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
                goal_yaw = theta + 0.5 * math.pi
                goal_yaw = wrap_pi(goal_yaw)
                goal_ompl.setYaw(goal_yaw)
                # TODO scoped vs abstract state
                # self._goal_states.addState(goal_ompl)
                self._goal_states.addState(goal_state)
                goal_yaw = theta - 0.5 * math.pi
                goal_yaw = wrap_pi(goal_yaw)
                goal_ompl.setYaw(goal_yaw)
                # Add additional state for bidirectional tangents
                # TODO scoped vs abstract state
                # self._goal_states.addState(goal_ompl)
                self._goal_states.addState(goal_state)

        self._problem_setup.setGoal(self._goal_states)
        self._problem_setup.setup()

    # setup using start position and velocity and goal position and velocity
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
        self._lower_bound = lower_bound
        self._upper_bound = upper_bound

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
        # TODO: make roi_ratio a property
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
        print(f"[TerrainOmplRrt] run solver")
        if self._problem_setup.solve(time_budget):
            # self._problem_setup.getSolutionPath().print(std::cout)
            # self._problem_setup.simplifySolution()
            # self._problem_setup.getSolutionPath().print(std::cout)
            self._problem_setup.getPlannerData(self._planner_data)
            self._solve_duration = self._problem_setup.getLastPlanComputationTime()

        else:
            print(f"[TerrainOmplRrt] Solution Not found")

        if self._problem_setup.haveExactSolutionPath():
            print(f"[TerrainOmplRrt] Found Exact solution!")
            self.solutionPathToPath(self._problem_setup.getSolutionPath(), path)
            return True

        return False

    def Solve2(
        self, time_budget: float, path: list[tuple[float, float, float]]
    ) -> bool:
        # TODO: test
        if self._problem_setup.solve(time_budget):
            print(f"[TerrainOmplRrt] Found solution:")
            # self._problem_setup.getSolutionPath().print(std::cout);
            # self._problem_setup.simplifySolution();
            # self._problem_setup.getSolutionPath().print(std::cout);
            self._problem_setup.getPlannerData(self._planner_data)
            self._solve_duration = self._problem_setup.getLastPlanComputationTime()

        else:
            print(f"[TerrainOmplRrt] Solution Not found")

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
        if dubins_path_type == DubinsPath.DubinsPathSegmentType.DUBINS_LEFT:
            segment_curvature = maximum_curvature
        elif dubins_path_type == DubinsPath.DubinsPathSegmentType.DUBINS_RIGHT:
            segment_curvature = -maximum_curvature
        elif dubins_path_type == DubinsPath.DubinsPathSegmentType.DUBINS_STRAIGHT:
            segment_curvature = 0.0

        return segment_curvature

    def solutionPathToPath(
        self,
        path: og.PathGeometric,
        trajectory_segments: Path,
        resolution: float = 0.05,
    ) -> None:
        # TODO: test
        print(f"[TerrainOmplRrt] convert solution path to Path")
        trajectory_segments.reset_segments()

        state_vector = path.getStates()
        print(f"[TerrainOmplRrt] path has {len(state_vector)} states")

        for idx in range(len(state_vector) - 1):
            # start and end of current segment
            from_state = state_vector[idx]
            to_state = state_vector[idx + 1]

            # state space
            da_space = self._problem_setup.getStateSpace()

            # TODO: remove debug prints
            debug_print = True
            if debug_print:
                da_from_state = DubinsAirplaneStateSpace.DubinsAirplaneState(from_state)
                da_to_state = DubinsAirplaneStateSpace.DubinsAirplaneState(to_state)
                print(f"[TerrainOmplRrt] state[{idx}]:    {da_from_state}")
                print(f"[TerrainOmplRrt] state[{idx + 1}]:    {da_to_state}")

            # NOTE: do not need to calculate the Dubins paths here
            #       as calculateSegments also calls dubins2
            # dubins_path = da_space.dubins2(from_state, to_state)
            (dubins_path, segmentStarts) = da_space.calculateSegments(
                from_state, to_state
            )

            if debug_print:

                def path_type_str(path_type):
                    msg = (
                        f"{dubins_path._type[0]}"
                        f" {dubins_path._type[1]}"
                        f" {dubins_path._type[2]}"
                    )
                    return msg

                print(f"[TerrainOmplRrt] dubins.idx:  {dubins_path.getIdx()}")
                print(
                    f"[TerrainOmplRrt] dubins.type: {path_type_str(dubins_path._type)}"
                )
                print(
                    f"[TerrainOmplRrt] dubins.len: "
                    f"{dubins_path._length[1]:.3f} "
                    f"{dubins_path._length[3]:.3f} "
                    f"{dubins_path._length[4]:.3f} "
                )
                print(f"[TerrainOmplRrt] dubins.alt:  {dubins_path.getAltitudeCase()}")
                print(
                    f"[TerrainOmplRrt] dubins.cls:  {dubins_path.getClassification()}"
                )
                print(f"[TerrainOmplRrt] dubins.ks:   {dubins_path._k_start}")
                print(f"[TerrainOmplRrt] dubins.ke:   {dubins_path._k_end}")

                print(f"[TerrainOmplRrt] segs[0]:     {segmentStarts.segmentStarts[0]}")
                print(f"[TerrainOmplRrt] segs[1]:     {segmentStarts.segmentStarts[1]}")
                print(f"[TerrainOmplRrt] segs[2]:     {segmentStarts.segmentStarts[2]}")
                print(f"[TerrainOmplRrt] segs[3]:     {segmentStarts.segmentStarts[3]}")
                print(f"[TerrainOmplRrt] segs[4]:     {segmentStarts.segmentStarts[4]}")
                print(f"[TerrainOmplRrt] segs[5]:     {segmentStarts.segmentStarts[5]}")

            segment_start_state = ob.State(da_space)
            segment_end_state = ob.State(da_space)

            total_length = dubins_path.length_2d()
            print(f"[TerrainOmplRrt] total_length: {total_length}")

            progress = 0.0
            for start_idx in range(len(segmentStarts.segmentStarts)):
                if dubins_path.getSegmentLength(start_idx) > 0.0:
                    segment_progress = (
                        dubins_path.getSegmentLength(start_idx) / total_length
                    )
                    # Read segment start and end states
                    TerrainOmplRrt.segmentStart2OmplState(
                        segmentStarts.segmentStarts[start_idx], segment_start_state
                    )
                    if (start_idx + 1) > (len(segmentStarts.segmentStarts) - 1):
                        segment_end_state = to_state
                    elif (start_idx + 1) > (
                        len(segmentStarts.segmentStarts) - 2
                    ) and dubins_path.getSegmentLength(start_idx + 1) == 0.0:
                        segment_end_state = to_state
                    else:
                        TerrainOmplRrt.segmentStart2OmplState(
                            segmentStarts.segmentStarts[start_idx + 1],
                            segment_end_state,
                        )

                    # Append to trajectory
                    trajectory = PathSegment()
                    trajectory.curvature = self.getSegmentCurvature(
                        self._problem_setup, dubins_path, start_idx
                    )
                    trajectory.flightpath_angle = dubins_path.getGamma()

                    state = ob.State(da_space)

                    # handle case when start and end state are the same
                    track_progress = 0.0
                    if total_length > 0.0:
                        # dt = resolution / total_length
                        num_step = int(total_length / resolution)
                        t_samples = np.linspace(
                            progress,
                            progress + segment_progress,
                            num_step,
                            endpoint=False,
                        )
                        for t in t_samples:
                            segment_state = path_segment.State()
                            # state = da_space.interpolate3(dubins_path, segmentStarts, t)
                            da_space.interpolate3(dubins_path, segmentStarts, t, state)

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
                            trajectory.append_state(segment_state)
                            track_progress = t
                    else:
                        track_progress = progress

                    # Append end state
                    if ((start_idx + 1) > (len(segmentStarts.segmentStarts) - 1)) or (
                        (start_idx + 1) > (len(segmentStarts.segmentStarts) - 2)
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
                        trajectory.append_state(end_state)

                    progress = track_progress
                    # Do not append trajectory if the segment is too short
                    if trajectory.state_count() > 1:
                        trajectory_segments.append_segment(trajectory)

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

        for state in state_vector:
            position = TerrainOmplRrt.dubinsairplanePosition(state)
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
    def dubinsairplanePosition(state: ob.State) -> tuple[float, float, float]:
        # TODO: test
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        position = (da_state.getX(), da_state.getY(), da_state.getZ())
        return position

    @staticmethod
    def dubinsairplaneYaw(state: ob.State) -> float:
        # TODO: test
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        yaw = da_state.getYaw()
        return yaw

    @staticmethod
    def segmentStart2OmplState(
        start: DubinsAirplaneStateSpace.SegmentStarts.Start, state: ob.State
    ) -> None:
        # TODO: test
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        da_state.setXYZYaw(start.x, start.y, start.x, start.yaw)

    def setAltitudeLimits(self, max_altitude: float, min_altitude: float) -> None:
        # TODO: test
        self._max_altitude = max_altitude
        self._min_altitude = min_altitude

    @staticmethod
    def validatePosition(
        map: GridMap, centre_pos: tuple[float, float, float], radius: float
    ) -> bool:
        """
        Check the position is not an inevitable collision state (ICS)

        NOTE: this is an approximate the check whether the position
              is a ICS.
        """
        # Map must have two layers: "distance_surface" and "max_elevation".
        min_layer = "distance_surface"
        max_layer = "max_elevation"
        num_steps = 24
        theta_samples = np.linspace(-math.pi, math.pi, num_steps)
        pos_2d = np.array([centre_pos[0], centre_pos[1]])
        pos_z = centre_pos[2]
        points = [
            pos_2d + radius * np.array((np.cos(theta), np.sin(theta)))
            for theta in theta_samples
        ]
        for point in points:
            min_z = map.atPosition(min_layer, point)
            max_z = map.atPosition(max_layer, point)
            if pos_z < min_z or pos_z > max_z:
                return False
        return True
