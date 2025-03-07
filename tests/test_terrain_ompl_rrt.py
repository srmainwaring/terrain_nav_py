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

import math
import pytest
import sys

from ompl import base as ob
from ompl import geometric as og


from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace

from terrain_nav_py.path_segment import PathSegment
from terrain_nav_py.path import Path

from terrain_nav_py.terrain_ompl import GridMap

from terrain_nav_py.terrain_ompl_rrt import TerrainMap
from terrain_nav_py.terrain_ompl_rrt import TerrainOmplRrt


def test_terrain_ompl_rrt():
    # create terrain map
    grid_map = GridMap()
    terrain_map = TerrainMap()
    terrain_map.setGridMap(grid_map)

    # create planner
    da_space = DubinsAirplaneStateSpace(turningRadius=60.0, gam=0.15)
    planner = TerrainOmplRrt(da_space)
    planner.setMap(terrain_map)
    planner.setAltitudeLimits(max_altitude=120.0, min_altitude=50.0)

    # initialise from map
    start_pos = (10.0, 20.0, 60.0)
    goal_pos = (100.0, 200.0, 100.0)
    start_loiter_radius = 60.0
    planner.setBoundsFromMap(terrain_map.getGridMap())

    # PLANNER_MODE.GLOBAL
    # set up problem from start and goal positions and start loiter radius
    print("PLANNER_MODE.GLOBAL")
    planner.setupProblem2(start_pos, goal_pos, start_loiter_radius)
    # candidate_path = Path()
    # planner.Solve1(time_budget=1.0, path=candidate_path)

    # PLANNER_MODE.EMERGENCY_ABORT
    # set up problem start position and velocity and rally points
    # planner.setupProblem4(start_pos, start_vel, rally_points)
    # planner_solution_path = Path()
    # planner.Solve(time_budget=1.0, path=planner_solution_path)

    # PLANNER_MODE.RETURN
    # set up problem start position and velocity and home position and radius
    # planner.setupProblem3(start_pos, start_vel, home_pos, home_pos_radius)
    # planner_solution_path = Path()
    # planner.Solve(time_budget=1.0, path=planner_solution_path)


def main():
    test_terrain_ompl_rrt()


if __name__ == "__main__":
    main()
