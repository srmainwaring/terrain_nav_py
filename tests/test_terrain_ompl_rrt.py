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

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace

from terrain_nav_py.grid_map import GridMap
from terrain_nav_py.grid_map import GridMapSRTM

from terrain_nav_py.path import Path

from terrain_nav_py.terrain_map import TerrainMap

from terrain_nav_py.terrain_ompl_rrt import TerrainOmplRrt


def test_terrain_ompl_rrt():
    # create terrain map
    # grid_map = GridMap()
    grid_map = GridMapSRTM(home_lat=56.6987387, home_lon=-6.1082210)
    terrain_map = TerrainMap()
    terrain_map.setGridMap(grid_map)

    # create planner
    da_space = DubinsAirplaneStateSpace(turningRadius=40.0, gam=0.06)
    planner = TerrainOmplRrt(da_space)
    planner.setMap(terrain_map)
    planner.setAltitudeLimits(max_altitude=120.0, min_altitude=50.0)

    # initialise from map
    start_pos = [10.0, 20.0, 60.0]
    goal_pos = [4200.0, -3000.0, 60.0]
    loiter_radius = 40.0
    planner.setBoundsFromMap(terrain_map.getGridMap())

    # adjust the start and goal altitudes
    start_pos[2] += grid_map.atPosition("elevation", start_pos)
    goal_pos[2] += grid_map.atPosition("elevation", goal_pos)

    # PLANNER_MODE.GLOBAL
    # set up problem from start and goal positions and start loiter radius
    print("PLANNER_MODE.GLOBAL")
    planner.setupProblem2(start_pos, goal_pos, loiter_radius)
    candidate_path = Path()
    planner.Solve1(time_budget=1.0, path=candidate_path)

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

    # only display plots if run as script
    if __name__ == "__main__":
        # solution path - og.PathGeometric
        solution_path = planner._problem_setup.getSolutionPath()
        states = solution_path.getStates()
        # state1 = solution_path.getState(0)
        # state2 = solution_path.getState(solution_path.getStateCount() - 1)
        # pos1 = TerrainOmplRrt.dubinsairplanePosition(state1)
        # yaw1 = TerrainOmplRrt.dubinsairplaneYaw(state1)
        # pos2 = TerrainOmplRrt.dubinsairplanePosition(state2)
        # yaw2 = TerrainOmplRrt.dubinsairplaneYaw(state2)
        print(f"Planner solution path (og.PathGeometric)")
        # print(type(solution_path))
        print(f"path length: {solution_path.length():.2f}")
        print(f"states count: {len(states)}")
        # print(f"pos1: {pos1}, yaw1: {yaw1}")
        # print(f"pos2: {pos2}, yaw2: {yaw2}")
        plot_path(start_pos, goal_pos, loiter_radius, candidate_path, states, grid_map)


def plot_path(
    start_pos, goal_pos, loiter_radius, path=None, states=None, grid_map=None
):
    import matplotlib.pyplot as plt
    import numpy as np

    def plot_circle(ax, position, radius, label=""):
        theta = np.arange(-math.pi, math.pi, 0.01 * math.pi)
        x = position[0] + radius * np.cos(theta)
        y = position[1] + radius * np.sin(theta)
        z = position[2] * np.ones(len(theta))
        ax.scatter(x, y, z, linestyle="solid", marker=".", s=1, c="blue")
        ax.text(position[0] + 1.5 * radius, position[1], position[2], label)

    def plot_path(ax, path):
        position = path.position()
        position = np.array(position)
        x = position[:, 0]
        y = position[:, 1]
        z = position[:, 2]
        ax.scatter(x, y, z, linestyle="solid", marker=".", s=1, c="green")

    def plot_state(ax, state, label=""):
        position = TerrainOmplRrt.dubinsairplanePosition(state)
        yaw = TerrainOmplRrt.dubinsairplaneYaw(state)
        x = position[0]
        y = position[1]
        z = position[2]
        ax.scatter(x, y, z, marker="v", s=48, c="red")
        ax.text(position[0], position[1], position[2], label)
        # TODO: remove debug prints 
        # print(position)

    def plot_states(ax, states):
        for i, state in enumerate(states):
            plot_state(ax, state, f"state{i}")

    def plot_terrain(ax, grid_map):
        length = grid_map.getLength()
        length_x = length[0]
        length_y = length[1]
        grid_spacing = 30
        x = np.arange(-0.5 * length_x, 0.5 * length_x, grid_spacing)
        y = np.arange(-0.5 * length_y, 0.5 * length_y, grid_spacing)
        x_grid, y_grid = np.meshgrid(x, y)

        def terrain_surface(x, y):
            alt = []
            for east in y:
                alt_y = []
                for north in x:
                    alt_y.append(grid_map.atPosition("elevation", (north, east)))
                alt.append(alt_y)
            return alt

        z_grid = np.array(terrain_surface(x, y))
        # ax.contour(x_grid, y_grid, z_grid, levels=10)
        ax.plot_wireframe(x_grid, y_grid, z_grid, linewidth=1, linestyle="solid", alpha=0.3, color="grey")
        # aspect ratio is 1:1:1 in data space
        ax.set_box_aspect((np.ptp(x_grid), np.ptp(y_grid), 5 * np.ptp(z_grid)))

    # setup plot
    ax = plt.figure().add_subplot(projection="3d")
    ax.set_xlim(-5000.0, 5000.0)
    ax.set_ylim(-5000.0, 5000.0)
    ax.set_zlim(0.0, 500.0)
    ax.set_xlabel("north (m)")
    ax.set_ylabel("east (m)")
    ax.set_title("Terrain Navigation, terrain source: SRTM1")
    ax.invert_yaxis()

    # start circle
    plot_circle(ax, start_pos, loiter_radius, "start")

    # goal circle
    plot_circle(ax, goal_pos, loiter_radius, "goal")

    # path
    # if path is not None:
    #     plot_path(ax, path)

    # states
    if states is not None:
        plot_states(ax, states)

    # map
    if grid_map is not None:
        plot_terrain(ax, grid_map)

    plt.show()


def main():
    test_terrain_ompl_rrt()


if __name__ == "__main__":
    main()
