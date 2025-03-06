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
    # create terrain maps
    grid_map = GridMap()
    terrain_map = TerrainMap()
    terrain_map.setGridMap(grid_map)

    pos = grid_map.getPosition()
    assert pos[0] == 0.0
    assert pos[1] == 0.0

    # create planner
    da_space  = DubinsAirplaneStateSpace(turningRadius=60.0, gam=0.15)
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
    # planner.setupProblem2(start_pos, goal_pos, start_loiter_radius)
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
