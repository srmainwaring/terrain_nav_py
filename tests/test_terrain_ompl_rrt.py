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

from terrain_nav_py import add_stderr_logger

from terrain_nav_py.path import Path

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace

from terrain_nav_py.grid_map import GridMapSRTM

from terrain_nav_py.path import Path

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
    map_lat=56.6987387
    map_lon=-6.1082210
    turningRadius=40.0
    gam=0.1
    start_pos = [0.0, 0.0, 60.0]
    goal_pos = [-200.0, 200.0, 60.0]
    max_altitude=120.0
    min_altitude=50.0

    state: [-40.0000, -0.0000, 73.9186; 1.5708]
    state: [-187.6393, 161.9577, 104.6088; -2.8274]
    """
    # configure loggers
    add_stderr_logger()
    add_test_stderr_logger()
    ou.setLogLevel(ou.LogLevel.LOG_DEBUG)
    print(ou.getLogLevel())

    map_lat = 56.6987387
    map_lon = -6.1082210
    gamma = 0.1
    loiter_radius = 40.0
    max_altitude = 120.0
    min_altitude = 50.0

    # create terrain map
    grid_map = GridMapSRTM(map_lat, map_lon)
    grid_map.setGridLength(800)
    terrain_map = TerrainMap()
    terrain_map.setGridMap(grid_map)

    # create planner manager
    da_space = DubinsAirplaneStateSpace(turningRadius=loiter_radius, gam=gamma)
    planner_mgr = TerrainOmplRrt(da_space)
    planner_mgr.setMap(terrain_map)
    planner_mgr.setAltitudeLimits(max_altitude, min_altitude)
    planner_mgr.setBoundsFromMap(terrain_map.getGridMap())

    # set start and goal
    start_pos = [0.0, 0.0, 60.0]
    goal_pos = [-200.0, 200.0, 60.0]

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
    state = ob.State(da_space)
    da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
    da_state.setXYZYaw(-40.0000, -0.0000, 73.9186, 1.5708)
    solution_path.append(state())

    state = ob.State(da_space)
    da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
    da_state.setXYZYaw(-187.6393, 161.9577, 104.6088, -2.8274)
    solution_path.append(state())

    trajectory_segments = Path()
    planner_mgr.solutionPathToPath(solution_path, trajectory_segments)

    states = solution_path.getStates()

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

        log.debug(f"position.shape: {position.shape}")
        log.debug(f"velocity.shape: {velocity.shape}")

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
    # ax.set_zlim(0.0, 500.0)
    ax.set_zlim(1000.0, 2500.0)
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
    # test_terrain_ompl_rrt_solution_path_to_path()


if __name__ == "__main__":
    main()
