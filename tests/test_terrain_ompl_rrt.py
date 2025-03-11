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
    grid_map.setGridLength(2500)

    terrain_map = TerrainMap()
    terrain_map.setGridMap(grid_map)

    # create planner
    da_space = DubinsAirplaneStateSpace(turningRadius=40.0, gam=0.15)
    planner = TerrainOmplRrt(da_space)
    planner.setMap(terrain_map)
    planner.setAltitudeLimits(max_altitude=120.0, min_altitude=50.0)

    # initialise from map (ENU)
    start_pos = [20.0, 10.0, 60.0]
    # goal_pos = [-3000.0, 4200.0, 60.0]
    # goal_pos = [-1500.0, 2200.0, 60.0]
    # goal_pos = [-4000.0, 100.0, 60.0]
    goal_pos = [-4500.0, 4000.0, 60.0]
    # grid_map.setGridLength(10000)

    # example
    # start_pos = [0.0, 0.0, 60.0]
    # goal_pos = [-1000.0, 500.0, 60.0]
    # grid_map.setGridLength(2500)

    start_pos = [0.0, 0.0, 60.0]
    goal_pos = [-400.0, 400.0, 60.0]
    grid_map.setGridLength(800)

    loiter_radius = 40.0
    planner.setBoundsFromMap(terrain_map.getGridMap())

    # adjust the start and goal altitudes
    start_pos[2] += grid_map.atPosition("elevation", start_pos)
    goal_pos[2] += grid_map.atPosition("elevation", goal_pos)

    # TODO: check the start and goal states are valid. This requires a
    #       correctly calculated distance surface: a surface where at each
    # point a circle of radius r will not intersect with the elevation layer.

    # NOTE: if the time budget is insufficient, the solution tree may not
    #       include a goal state, and an approximate solution will be found.

    # PLANNER_MODE.GLOBAL
    # set up problem from start and goal positions and start loiter radius
    print("PLANNER_MODE.GLOBAL")
    planner.setupProblem2(start_pos, goal_pos, loiter_radius)
    candidate_path = Path()
    planner.Solve1(time_budget=15.0, path=candidate_path)

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
        plot_path(
            start_pos, goal_pos, loiter_radius, candidate_path, states, grid_map
        )


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
        # check the state vector is not empty
        if position.size == 0:
            return
        x = position[:, 0]
        y = position[:, 1]
        z = position[:, 2]
        ax.scatter(x, y, z, linestyle="solid", marker=".", s=1, c="green")

        # plot velocity vectors along the path
        velocity = path.velocity()
        velocity = np.array(velocity)

        print(f"position.shape: {position.shape}")
        print(f"velocity.shape: {velocity.shape}")

        scale = 0.25 * loiter_radius
        stride = 10
        vx = velocity[:, 0]
        vy = velocity[:, 1]
        vz = velocity[:, 2]
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
        x = np.arange(-0.5 * length_x, 0.5 * length_x, grid_spacing)
        y = np.arange(-0.5 * length_y, 0.5 * length_y, grid_spacing)
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
    ax.set_zlim(0.0, 500.0)
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
    test_terrain_ompl_rrt()


if __name__ == "__main__":
    main()
