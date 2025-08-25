"""
Multi-processing terrain planner
"""

import math
import time
import threading

from dataclasses import dataclass, field

from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import mp_util

# terrain navigation
from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace
from terrain_nav_py.grid_map import GridMapSRTM
from terrain_nav_py.path import Path
from terrain_nav_py.terrain_map import TerrainMap
from terrain_nav_py.terrain_ompl_rrt import TerrainOmplRrt

# data messages [in]


@dataclass
class PlannerStartLatLon:
    start_latlon: tuple[float, float] = None
    is_valid: bool = False


@dataclass
class PlannerGoalLatLon:
    goal_latlon: tuple[float, float] = None
    is_valid: bool = False


@dataclass
class PlannerLoiterAglAlt:
    loiter_agl_alt: float


@dataclass
class PlannerLoiterRadius:
    loiter_radius: float


@dataclass
class PlannerTurningRadius:
    turning_radius: float


@dataclass
class PlannerClimbAngleDeg:
    climb_angle_deg: float


@dataclass
class PlannerMaxAglAlt:
    max_agl_alt: float


@dataclass
class PlannerMinAglAlt:
    min_agl_alt: float


@dataclass
class PlannerGridLatLon:
    grid_latlon: tuple[float, float] = None


@dataclass
class PlannerGridSpacing:
    grid_spacing: float


@dataclass
class PlannerGridLength:
    grid_length: float


@dataclass
class PlannerTerrainSource:
    terrain_source: str


@dataclass
class PlannerTimeBudget:
    time_budget: float


@dataclass
class PlannerResolution:
    resolution: float


@dataclass
class PlannerPolyFences:
    exclusion_polygons: list = field(default_factory=list)
    inclusion_polygons: list = field(default_factory=list)
    exclusion_circles: list = field(default_factory=list)
    inclusion_circles: list = field(default_factory=list)


# data messages [out]


@dataclass
class PlannerStatus:
    status: str


@dataclass
class PlannerPath:
    path: Path = field(default_factory=Path)


@dataclass
class PlannerStates:
    states: list = field(default_factory=list)


# command messages


@dataclass
class PlannerCmdRunPlanner:
    pass


class TerrainPlanner(multiproc.Process):
    # NOTE: the planner process cannot be daemonic, because it prevents
    #       the terrain tile downloader from running (daemonic processes
    #        are not alloed to have children).
    def __init__(self, pipe_send, pipe_recv, close_event):
        super().__init__(name="TerrainPlanner", daemon=False)

        self._pipe_send = pipe_send
        self._pipe_recv = pipe_recv
        self._close_event = close_event

        # thread to process incoming messages
        self._message_thread = None
        self._lock = multiproc.Lock()

        # *** process state ***
        self._do_init_terrain_map = False
        self._do_init_planner = False
        self._do_update_start_pos = False
        self._do_update_goal_pos = False
        self._do_run_planner = False

        # *** planner settings ***
        self._loiter_agl_alt = 60.0
        self._loiter_radius = 60.0
        self._turning_radius = 60.0
        self._climb_angle_deg = 8.0
        self._max_agl_alt = 100.0
        self._min_agl_alt = 50.0
        self._grid_spacing = 30.0
        self._grid_length = 10000.0
        self._terrain_source = "SRTM1"
        self._time_budget = 20.0
        self._resolution = 100.0

        # *** planner state ***
        self._start_latlon = (None, None)
        self._start_pos_enu = (None, None)
        self._start_is_valid = False
        self._goal_latlon = (None, None)
        self._goal_pos_enu = (None, None)
        self._goal_is_valid = False
        self._grid_map = None
        self._grid_map_lat = None
        self._grid_map_lon = None
        self._terrain_map = None
        self._da_space = None
        self._planner_mgr = None

        # *** fences ***
        self._exclusion_polygons = []
        self._inclusion_polygons = []
        self._exclusion_circles = []
        self._inclusion_circles = []

    def run(self):
        # start threads
        self.start_message_thread()

        # monitor events
        while True:
            # check for close event
            if self._close_event.is_set():
                break

            try:
                # copy shared state
                self._lock.acquire()
                do_init_terrain_map = self._do_init_terrain_map
                do_init_planner = self._do_init_planner
                do_update_start_pos = self._do_update_start_pos
                do_update_goal_pos = self._do_update_goal_pos
                do_run_planner = self._do_run_planner
                self._lock.release()

                # run planner operations
                if do_init_terrain_map:
                    self.init_terrain_map()

                    self._lock.acquire()
                    self._do_init_terrain_map = False
                    self._lock.release()

                if do_init_planner:
                    self.init_planner()

                    self._lock.acquire()
                    self._do_init_planner = False
                    self._lock.release()

                if do_update_start_pos:
                    self._lock.acquire()
                    (lat, lon) = self._start_latlon
                    self._lock.release()

                    self.set_start_pos_enu(lat, lon)

                    self._lock.acquire()
                    self._do_update_start_pos = False
                    self._lock.release()

                if do_update_goal_pos:
                    self._lock.acquire()
                    (lat, lon) = self._goal_latlon
                    self._lock.release()

                    self.set_goal_pos_enu(lat, lon)

                    self._lock.acquire()
                    self._do_update_goal_pos = False
                    self._lock.release()

                if do_run_planner:
                    self.run_planner()

                    self._lock.acquire()
                    self._do_run_planner = False
                    self._lock.release()

            except ValueError as e:
                print(f"[TerrainPlanner] {e}")
                self._lock.release()
            except Exception as e:
                print(f"[TerrainPlanner] exception in main loop, stopping: {e}")
                self._lock.release()
                break

            time.sleep(0.01)

    def start_message_thread(self):
        if self._message_thread:
            return

        t = threading.Thread(target=self.process_messages, name="MessageThread")
        t.daemon = True
        self._message_thread = t
        t.start()

    def process_messages(self):
        """
        Process incoming messages
        """
        while True:
            # receive data from parent process
            while self._pipe_recv.poll():
                msg = self._pipe_recv.recv()

                if isinstance(msg, PlannerStartLatLon):
                    self.on_start_lat_lon(msg)
                elif isinstance(msg, PlannerGoalLatLon):
                    self.on_goal_lat_lon(msg)
                elif isinstance(msg, PlannerLoiterAglAlt):
                    self.on_loiter_agl_alt(msg)
                elif isinstance(msg, PlannerLoiterRadius):
                    self.on_loiter_radius(msg)
                elif isinstance(msg, PlannerTurningRadius):
                    self.on_turning_radius(msg)
                elif isinstance(msg, PlannerClimbAngleDeg):
                    self.on_climb_angle_deg(msg)
                elif isinstance(msg, PlannerMaxAglAlt):
                    self.on_max_agl_alt(msg)
                elif isinstance(msg, PlannerMinAglAlt):
                    self.on_min_agl_alt(msg)
                elif isinstance(msg, PlannerGridLatLon):
                    self.on_grid_latlon(msg)
                elif isinstance(msg, PlannerGridSpacing):
                    self.on_grid_spacing(msg)
                elif isinstance(msg, PlannerGridLength):
                    self.on_grid_length(msg)
                elif isinstance(msg, PlannerTerrainSource):
                    self.on_terrain_source(msg)
                elif isinstance(msg, PlannerTimeBudget):
                    self.on_time_budget(msg)
                elif isinstance(msg, PlannerResolution):
                    self.on_resolution(msg)
                elif isinstance(msg, PlannerPolyFences):
                    self.on_polyfences(msg)
                elif isinstance(msg, PlannerCmdRunPlanner):
                    self.on_cmd_run_planner(msg)

            # update at 100 Hz
            time.sleep(0.01)

    def on_start_lat_lon(self, msg):
        self._lock.acquire()
        self._start_latlon = msg.start_latlon
        self._do_update_start_pos = True
        self._lock.release()

    def on_goal_lat_lon(self, msg):
        self._lock.acquire()
        self._goal_latlon = msg.goal_latlon
        self._do_update_goal_pos = True
        self._lock.release()

    def on_loiter_agl_alt(self, msg):
        self._lock.acquire()
        self._loiter_agl_alt = msg.loiter_agl_alt
        self._do_update_start_pos = True
        self._do_update_goal_pos = True
        self._lock.release()

    def on_loiter_radius(self, msg):
        self._lock.acquire()
        self._loiter_radius = msg.loiter_radius
        self._do_init_planner = True
        self._do_update_start_pos = True
        self._lock.release()
        # TODO: recalculate start and goal positions

    def on_turning_radius(self, msg):
        self._lock.acquire()
        self._turning_radius = msg.turning_radius
        self._do_init_planner = True
        self._do_update_goal_pos = True
        self._lock.release()

    def on_climb_angle_deg(self, msg):
        self._lock.acquire()
        self._climb_angle_deg = msg.climb_angle_deg
        self._do_init_planner = True
        self._lock.release()

    def on_max_agl_alt(self, msg):
        self._lock.acquire()
        self._max_agl_alt = msg.max_agl_alt
        self._do_init_planner = True
        self._lock.release()

    def on_min_agl_alt(self, msg):
        self._lock.acquire()
        self._min_agl_alt = msg.min_agl_alt
        self._do_init_planner = True
        self._lock.release()

    def on_grid_latlon(self, msg):
        self._lock.acquire()
        self._grid_map_lat = msg.grid_latlon[0]
        self._grid_map_lon = msg.grid_latlon[1]
        self._do_init_terrain_map = True
        self._do_init_planner = True
        self._do_update_start_pos = True
        self._do_update_goal_pos = True
        self._lock.release()

    def on_grid_spacing(self, msg):
        self._lock.acquire()
        self._grid_spacing = msg.grid_spacing
        self._do_init_terrain_map = True
        self._do_init_planner = True
        self._lock.release()

    def on_grid_length(self, msg):
        self._lock.acquire()
        self._grid_length = msg.grid_length
        self._do_init_terrain_map = True
        self._do_init_planner = True
        self._lock.release()

    def on_terrain_source(self, msg):
        self._lock.acquire()
        self._terrain_source = msg.terrain_source
        self._do_init_terrain_map = True
        self._do_init_planner = True
        self._lock.release()

    def on_time_budget(self, msg):
        self._lock.acquire()
        self._time_budget = msg.time_budget
        self._lock.release()

    def on_resolution(self, msg):
        self._lock.acquire()
        self._resolution = msg.resolution
        self._do_init_planner = True
        self._lock.release()

    def on_polyfences(self, msg):
        self._lock.acquire()
        self._exclusion_polygons = msg.exclusion_polygons
        self._inclusion_polygons = msg.inclusion_polygons
        self._exclusion_circles = msg.exclusion_circles
        self._inclusion_circles = msg.inclusion_circles
        self._do_init_planner = True
        self._lock.release()

    def on_cmd_run_planner(self, msg):
        self._lock.acquire()
        self._do_run_planner = True
        self._lock.release()

    def init_terrain_map(self):
        if not self.have_gridmap_latlon():
            return

        self._lock.acquire()

        self._grid_map = GridMapSRTM(
            map_lat=self._grid_map_lat, map_lon=self._grid_map_lon
        )
        self._grid_map.setGridSpacing(self._grid_spacing)
        self._grid_map.setGridLength(self._grid_length)
        self._grid_map.setTerrainSource(self._terrain_source)

        # TODO: set up distance layer (too slow in current version)
        # if self.is_debug:
        #     print(f"[TerrainPlanner] calculating distance-surface...", end="")
        # self._grid_map.addLayerDistanceTransform(surface_distance=self.terrainnav_settings.min_agl_alt)

        self._terrain_map = TerrainMap()
        self._terrain_map.setGridMap(self._grid_map)

        self._lock.release()

    def init_planner(self):
        # NOTE: initialisation ordering
        #
        # - create the state space
        # - set the map
        # - set altitude limits
        # - set bounds
        # - configureProblem:
        #   requires:
        #     - map
        #     - bounds (altitude limits)
        #   creates:
        #     - default planner
        #     - default objective
        #     - terrain collision validatity checker
        #     - planner data
        # - set start and goal states
        # - setup problem
        #   - (re-runs configureProblem internally)

        self._lock.acquire()

        # check the terrain map has been initialised
        if self._terrain_map is None:
            self._lock.release()
            return

        # recreate planner, as inputs may change
        self._da_space = DubinsAirplaneStateSpace(
            turningRadius=self._turning_radius,
            gam=math.radians(self._climb_angle_deg),
        )
        self._planner_mgr = TerrainOmplRrt(self._da_space)
        self._planner_mgr.setMap(self._terrain_map)
        self._planner_mgr.setAltitudeLimits(
            max_altitude=self._max_agl_alt,
            min_altitude=self._min_agl_alt,
        )
        self._planner_mgr.setBoundsFromMap(self._terrain_map.getGridMap())

        # run initial configuration so we can finish setting up fences etc.
        self._planner_mgr.configureProblem()

        # update problem
        problem = self._planner_mgr.getProblemSetup()

        # set fences - must called be after configureProblem
        problem.setExclusionPolygons(
            TerrainPlanner.polyfences_polygon_to_enu(
                self._grid_map_lat, self._grid_map_lon, self._exclusion_polygons
            )
        )
        problem.setInclusionPolygons(
            TerrainPlanner.polyfences_polygon_to_enu(
                self._grid_map_lat, self._grid_map_lon, self._inclusion_polygons
            )
        )
        problem.setExclusionCircles(
            TerrainPlanner.polyfences_circle_to_enu(
                self._grid_map_lat, self._grid_map_lon, self._exclusion_circles
            )
        )
        problem.setInclusionCircles(
            TerrainPlanner.polyfences_circle_to_enu(
                self._grid_map_lat, self._grid_map_lon, self._inclusion_circles
            )
        )

        # adjust validity checking resolution
        resolution_requested = self._resolution / self._grid_length
        problem.setStateValidityCheckingResolution(resolution_requested)

        self._lock.release()

    def run_planner(self):
        self._lock.acquire()

        # check start position is valid
        if not self._start_is_valid:
            msg = PlannerStatus(status="INVALID_START")
            self._pipe_send.send(msg)
            self._lock.release()
            return

        # check goal position is valid
        if not self._goal_is_valid:
            msg = PlannerStatus(status="INVALID_GOAL")
            self._pipe_send.send(msg)
            self._lock.release()
            return

        # set up problem and run
        self._planner_mgr.setupProblem2(
            self._start_pos_enu,
            self._goal_pos_enu,
            self._loiter_radius,
        )

        # run the solver
        candidate_path = Path()
        try:
            # NOTE: calculation in progress
            msg = PlannerStatus(status="PENDING")

            self._planner_mgr.Solve1(
                time_budget=self._time_budget,
                path=candidate_path,
            )
        except RuntimeError as e:
            # TODO: append error message
            msg = PlannerStatus(status="PLANNER_EXCEPTION")
            self._pipe_send.send(msg)
            self._lock.release()
            return

        # return if no solution
        if not self._planner_mgr.getProblemSetup().haveSolutionPath():
            msg = PlannerStatus(status="NO_SOLUTION")
            self._pipe_send.send(msg)
            self._lock.release()
            return

        # TODO: replace string with an enum class for planner status
        msg = PlannerStatus(status="OK")
        self._pipe_send.send(msg)

        # send path
        msg = PlannerPath(path=candidate_path)
        self._pipe_send.send(msg)

        # send states
        # NOTE: cannot send ompl states (cannot be pickled)
        solution_path = self._planner_mgr.getProblemSetup().getSolutionPath()
        ompl_states = solution_path.getStates()
        states = []
        for state in ompl_states:
            pos = TerrainOmplRrt.dubinsairplanePosition(state)
            yaw = TerrainOmplRrt.dubinsairplaneYaw(state)
            states.append([pos[0], pos[1], pos[2], yaw])
        msg = PlannerStates(states=states)
        self._pipe_send.send(msg)

        self._lock.release()

    def have_gridmap_latlon(self):
        # check the grid lat and lon have been set
        self._lock.acquire()
        result = self._grid_map_lat is not None and self._grid_map_lon is not None
        self._lock.release()
        return result

    def set_start_pos_enu(self, lat, lon):
        if lat is None or lon is None or not self.have_gridmap_latlon():
            return

        self._lock.acquire()

        # calculate position (ENU)
        (east, north) = TerrainPlanner.latlon_to_enu(
            self._grid_map_lat, self._grid_map_lon, lat, lon
        )

        # adjust the altitudes above terrain
        try:
            elevation = self._grid_map.atPosition("elevation", (east, north))
        except ValueError as e:
            print(f"[TerrainPlanner] unable to set start position: {e}")
            self._lock.release()
            return

        self._start_pos_enu = [
            east,
            north,
            elevation + self._loiter_agl_alt,
        ]

        # check valid
        radius = self._loiter_radius
        self._start_is_valid = self._planner_mgr.validateCircle(
            self._start_pos_enu, radius
        )

        # send validated position
        self._pipe_send.send(PlannerStartLatLon((lat, lon), self._start_is_valid))

        self._lock.release()

    def set_goal_pos_enu(self, lat, lon):
        if lat is None or lon is None or not self.have_gridmap_latlon():
            return

        self._lock.acquire()

        # calculate position (ENU)
        (east, north) = TerrainPlanner.latlon_to_enu(
            self._grid_map_lat, self._grid_map_lon, lat, lon
        )

        # adjust the altitudes above terrain
        try:
            elevation = self._grid_map.atPosition("elevation", (east, north))
        except ValueError as e:
            print(f"[TerrainPlanner] unable to set goal position: {e}")
            self._lock.release()
            return

        self._goal_pos_enu = [
            east,
            north,
            elevation + self._loiter_agl_alt,
        ]

        # check valid
        radius = self._turning_radius
        self._goal_is_valid = self._planner_mgr.validateCircle(
            self._goal_pos_enu, radius
        )

        # send validated position
        self._pipe_send.send(PlannerGoalLatLon((lat, lon), self._goal_is_valid))

        self._lock.release()

    @staticmethod
    def latlon_to_enu(origin_lat, origin_lon, lat, lon):
        distance = mp_util.gps_distance(origin_lat, origin_lon, lat, lon)
        bearing_deg = mp_util.gps_bearing(origin_lat, origin_lon, lat, lon)
        bearing_rad = math.radians(bearing_deg)
        east = distance * math.sin(bearing_rad)
        north = distance * math.cos(bearing_rad)
        return (east, north)

    @staticmethod
    def polyfences_polygon_to_enu(origin_lat, origin_lon, polygons):
        """
        Convert polyfences polygones to ENU point polygons.

        :param origin_lat: latitude of the grid map origin
        :type origin_lat: float
        :param origin_lon: longitude of the grid map origin
        :type origin_lon: float
        :param polygons: list of MAVLink polyfences
        :return: list of polygons in ENU frame
        :rtype" list[list[tuple[float, float]]]
        """
        polygons_enu = []
        for polygon in polygons:
            points_enu = []
            for point in polygon:
                lat = point.x
                lon = point.y
                if point.get_type() == "MISSION_ITEM_INT":
                    lat *= 1e-7
                    lon *= 1e-7
                point_enu = TerrainPlanner.latlon_to_enu(
                    origin_lat, origin_lon, lat, lon
                )
                points_enu.append(point_enu)
            polygons_enu.append(points_enu)
        return polygons_enu

    @staticmethod
    def polyfences_circle_to_enu(origin_lat, origin_lon, circles):
        circles_enu = []
        for circle in circles:
            lat = circle.x
            lon = circle.y
            if circle.get_type() == "MISSION_ITEM_INT":
                lat *= 1e-7
                lon *= 1e-7
            (east, north) = TerrainPlanner.latlon_to_enu(
                origin_lat, origin_lon, lat, lon
            )
            radius = circle.param1
            circles_enu.append((east, north, radius))
        return circles_enu
