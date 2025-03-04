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

from ompl import base as ob
from ompl import geometric as og

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace
from terrain_nav_py.dubins_path import DubinsPath
from terrain_nav_py.path import Path
from terrain_nav_py.path import PathSegment
from terrain_nav_py.ompl_setup import OmplSetup
from terrain_nav_py.terrain_ompl import GridMap


class TerrainOmplRrt:
    """
    Setup and run the planning problem.
    """

    def __init__(self, space: ob.StateSpace):
        # TODO: test
        # TODO: locate definition of TerrainMap
        # self._minimum_turning_radius: float = 66.67
        self._problem_setup: OmplSetup = None
        self._map  #: TerrainMap = None
        self._min_altitude: float = 50.0
        self._max_altitude: float = 120.0
        self._planner_data: ob.PlannerData = None
        self._goal_states: ob.GoalStates = None
        self._lower_bound = (0.0, 0.0, 0.0)
        self._upper_bound = (0.0, 0.0, 0.0)
        self._solve_duration: float = 0.0
        self._check_max_altitude: bool = True

    # /**
    #   * @brief Configure OMPL problem descriptions
    #   *
    #   */
    # void configureProblem();
    def configureProblem(self) -> None:
        # TODO: test
        # TODO: implement
        pass

    # /**
    #   * @brief Setup problem with center position of start and goal loiter circles
    #   *
    #   * @param start_pos center of the start loiter position
    #   * @param goal center of the goal loiter position
    #   * @param start_loiter_radius Specify direction of the start circle.
    #   *          - Positive: anti clockwise
    #   *          - Negative: Clockwise
    #   */
    # void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal) {
    def setupProblem(
        self, start_pos: tuple[float, float, float], goal: tuple[float, float, float]
    ) -> None:
        # TODO: test
        # TODO: implement
        pass
        #   this->setupProblem(
        #       start_pos, goal,
        #       problem_setup_->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->getMinTurningRadius());
        # };

    # /**
    #   * @brief Setup problem with center position of start and goal loiter circle with specific radius
    #   *
    #   * @param start_pos
    #   * @param goal
    #   * @param start_loiter_radius
    #   */
    # void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal, double start_loiter_radius);
    def setupProblem(
        self,
        start_pos: tuple[float, float, float],
        goal: tuple[float, float, float],
        start_loiter_radius: float,
    ) -> None:
        # TODO: test
        # TODO: implement
        pass

    # /**
    #   * @brief Setup problem with position, velocity of the start and center of the goal loiter circle
    #   *
    #   * @param start_pos position of the start state
    #   * @param start_vel velocity of the start state
    #   * @param goal center of the goal loiter position
    #   */
    # void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& goal,
    #                   double goal_radius = -1);
    def setupProblem(
        self,
        start_pos: tuple[float, float, float],
        start_vel: tuple[float, float, float],
        goal: tuple[float, float, float],
        goal_radius: float = -1.0,
    ) -> None:
        # TODO: test
        # TODO: implement
        pass

    # /**
    #   * @brief Setup problem with position, velocity of the start and goal state
    #   *
    #   * @param start_pos position of the start state
    #   * @param start_vel velocity of the start state
    #   * @param goal position of the goal state
    #   * @param goal_vel velocity of the goal state
    #   */
    # void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
    #                   const std::vector<Eigen::Vector3d>& goal_positions);
    def setupProblem(
        self,
        start_pos: tuple[float, float, float],
        start_vel: tuple[float, float, float],
        goal_positions: list[tuple[float, float, float]],
    ) -> None:
        # TODO: test
        # TODO: implement
        pass

    # /**
    #   * @brief Setup problem with position, velocity of the start and goal state
    #   *
    #   * @param start_pos position of the start state
    #   * @param start_vel velocity of the start state
    #   * @param goal position of the goal state
    #   * @param goal_vel velocity of the goal state
    #   */
    # void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& goal,
    #                   const Eigen::Vector3d& goal_vel);
    def setupProblem(
        self,
        start_pos: tuple[float, float, float],
        start_vel: tuple[float, float, float],
        goal: tuple[float, float, float],
        goal_vel: tuple[float, float, float],
    ) -> None:

        # TODO: test
        # TODO: implement
        pass

    # /**
    #   * @brief Set the Bounds of the statespace for the planner
    #   *
    #   * @param lower_bound lower bo
    #   * @param upper_bound
    #   */
    # void setBounds(const Eigen::Vector3d& lower_bound, const Eigen::Vector3d& upper_bound) {
    #   lower_bound_ = lower_bound;
    #   upper_bound_ = upper_bound;
    # }
    def setBounds(
        self,
        lower_bound: tuple[float, float, float],
        upper_bound: tuple[float, float, float],
    ) -> None:
        # TODO: test
        # TODO: implement
        pass

    # /**
    #   * @brief Set the Bounds of the statespace for the planner using the map
    #   *
    #   * @param map
    #   */
    # void setBoundsFromMap(const grid_map::GridMap& map);
    def setBoundsFromMap(self, map: GridMap) -> None:
        # TODO: test
        # TODO: implement
        pass

    # /**
    #   * @brief Set the Map
    #   *
    #   * @param map
    #   */
    # void setMap(std::shared_ptr<TerrainMap> map) { map_ = std::move(map); }
    def setMap(self, map) -> None:
        # TODO: test
        # TODO: implement
        pass

    # /**
    #   * @brief Set the Max Altitude Collision Check
    #   *
    #   * @param check_max_altitude If true, enables the maximum altitude collision checks
    #   */
    # void setMaxAltitudeCollisionChecks(bool check_max_altitude) { check_max_altitude_ = check_max_altitude; }
    def setMaxAltitudeCollisionChecks(self, check_max_altitude: bool) -> None:
        # TODO: test
        self._check_max_altitude = check_max_altitude

    # /**
    #   * @brief Solve the planning problem for a given time budget, and return a TerrainSegments object if an exact solution
    #   * is found
    #   *
    #   * @param time_budget [s] time the planner should use for planning
    #   * @param path
    #   * @return true Found exact solution
    #   * @return false Did not find an exact solution
    #   */
    # bool Solve(double time_budget, Path& path);
    def Solve(self, time_budget: float, path: Path) -> None:
        # TODO: test
        # TODO: implement
        pass

    # bool Solve(double time_budget, std::vector<Eigen::Vector3d>& path);
    def Solve(self, time_budget: float, path: list[tuple[float, float, float]]) -> None:
        # TODO: test
        # TODO: implement
        pass

    # double getSegmentCurvature(std::shared_ptr<ompl::OmplSetup> problem_setup,
    #                             fw_planning::spaces::DubinsPath& dubins_path, const size_t start_idx) const;
    def getSegmentCurvature(
        self, problem_setup: OmplSetup, dubins_path: DubinsPath, start_idx: int
    ) -> float:
        # TODO: test
        # TODO: implement
        pass

    # void solutionPathToPath(ompl::geometric::PathGeometric path, Path& trajectory_segments,
    #                         double resolution = 0.05) const;
    def solutionPathToPath(
        self,
        path: og.PathGeometric,
        trajectory_segments: Path,
        resolution: float = 0.05,
    ) -> None:
        # TODO: test
        # TODO: implement
        pass

    # void solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric path,
    #                                     std::vector<Eigen::Vector3d>& trajectory_points) const;
    def solutionPathToTrajectoryPoints(
        self,
        path: og.PathGeometric,
        trajectory_points: list[tuple[float, float, float]],
    ) -> None:
        # TODO: test
        # TODO: implement
        pass

    # std::shared_ptr<ompl::base::PlannerData> getPlannerData() { return planner_data_; };
    def getPlannerData(self) -> ob.PlannerData:
        # TODO: test
        # TODO: implement
        return self._planner_data

    # std::shared_ptr<ompl::OmplSetup> getProblemSetup() { return problem_setup_; };
    def getProblemSetup(self) -> OmplSetup:
        # TODO: test
        # TODO: implement
        return self._problem_setup

    # ompl::base::StateSamplerPtr allocTerrainStateSampler(const ompl::base::StateSpace* space) {
    #   return std::make_shared<ompl::TerrainStateSampler>(space, map_->getGridMap(), max_altitude_, min_altitude_);
    # }
    def allocTerrainStateSampler(self, space: ob.StateSpace) -> ob.StateSampler:
        # TODO: test
        # TODO: implement
        pass

    # bool getSolutionPathLength(double& path_length);
    def getSolutionPathLength(self) -> tuple[bool, float]:
        # TODO: test
        # TODO: implement
        result: bool = False
        path_length: float = 0.0
        return (result, path_length)

    # bool getSolutionPath(std::vector<Eigen::Vector3d>& path);
    def getSolutionPath(self) -> list[tuple[float, float, float]]:
        # TODO: test
        # TODO: implement
        pass

    # double getSolutionTime() { return solve_duration_; };
    def getSolutionTime(self) -> float:
        # TODO: test
        return self._solve_duration

    # static Eigen::Vector3d dubinsairplanePosition(ompl::base::State* state_ptr) {
    #   Eigen::Vector3d position(state_ptr->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(),
    #                             state_ptr->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY(),
    #                             state_ptr->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ());
    #   return position;
    # }
    @staticmethod
    def dubinsairplanePosition(state_ptr: ob.State) -> tuple[float, float, float]:
        # TODO: test
        # TODO: implement
        pass

    # static double dubinsairplaneYaw(ompl::base::State* state_ptr) {
    #   double yaw = state_ptr->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getYaw();
    #   return yaw;
    # }
    @staticmethod
    def dubinsairplaneYaw(state_ptr: ob.State) -> float:
        # TODO: test
        # TODO: implement
        pass

    # static inline void segmentStart2omplState(fw_planning::spaces::DubinsAirplaneStateSpace::SegmentStarts::Start start,
    #                                           ompl::base::State* state) {
    #   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(start.x);
    #   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(start.y);
    #   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(start.z);
    #   state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(start.yaw);
    # }
    @staticmethod
    def segmentStart2omplState(
        start: DubinsAirplaneStateSpace.SegmentStarts.Start, state: ob.State
    ) -> None:
        # TODO: test
        # TODO: implement
        pass

    # void setAltitudeLimits(const double max_altitude, const double min_altitude) {
    #   max_altitude_ = max_altitude;
    #   min_altitude_ = min_altitude;
    # }
    def setAltitudeLimits(self, max_altitude: float, min_altitude: float) -> None:
        # TODO: test
        # TODO: implement
        pass
