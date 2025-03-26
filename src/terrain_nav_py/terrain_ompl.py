# BSD 3-Clause License
#
# Copyright (c) 2025, Rhys Mainwaring
#
# Ported to Python from original C++ code in
# https://github.com/ethz-asl/terrain-navigation.git
#
# Copyright (c) 2021 Jaeyoung Lim, Autonomous Systems Lab,
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
Terrain planner state validity checker and sampler
"""

import logging
import math

from ompl import base as ob
from ompl import util as ou

from shapely import geometry

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace
from terrain_nav_py.grid_map import GridMap

log = logging.getLogger(__name__)


class TerrainValidityChecker(ob.StateValidityChecker):
    def __init__(
        self, space_info: ob.SpaceInformation, map: GridMap, check_max_altitude: bool
    ):
        super().__init__(space_info)
        self._map = map
        self._check_collision_max_altitude = check_max_altitude

        self._exclusion_polygons = []
        self._inclusion_polygons = []
        self._inclusion_circles = []
        self._exclusion_circles = []

    def isValid(self, state: ob.State) -> bool:
        """
        State validity check - overrides virual base class method

        :param state: the state to check
        :type state: ob.State
        """
        # TODO test
        # NOTE: DubinsAirplaneStateSpace
        # da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        # position = da_state.getXYZ()

        # TODO: OwenStateSpace only
        da_state = state
        position = (da_state[0], da_state[1], da_state[2])

        # NOTE: leave for in-depth debugging
        # log.debug(f"position: {position}")
        return not self.checkCollision(position)

    def checkCollision(self, position: tuple[float, float, float]) -> bool:
        """
        Check collisions on gridmap

        :param position: the position to check
        :return bool: True if the position collides with the terrain or boundary.
        """
        # TODO test
        in_collision = self.isInCollision("distance_surface", position, True)
        in_collision_max = False
        if self._check_collision_max_altitude:
            in_collision_max = self.isInCollision("max_elevation", position, False)

        return in_collision or in_collision_max

    def isInCollision(
        self, layer: str, position: tuple[float, float, float], is_above: bool
    ) -> bool:
        """
        Check if the state is in collision with a layer or fence

        :param layer: Name of the layer in the elevation map
        :type layer: str
        :param position: Position of the state to evaluate
        :type position: tuple[float, float, float]
        :param is_above: `True` if to check above layer, `False` to check below
        :type is_above: bool
        :return: `True` if the positon is in collision
        :rtype: bool
        """
        # check fences, circles are fastest so do first
        point = geometry.Point(position[0], position[1])
        if self.isInsideExclusionCircles(point):
            return True

        if self.isOutsideInclusionCircles(point):
            return True

        if self.isInsideExclusionPolygons(point):
            return True

        if self.isOutsideInclusionPolygons(point):
            return True

        # check elevation
        position_2d = (position[0], position[1])
        if self._map.isInside(position_2d):
            elevation = self._map.atPosition(layer, position_2d)
            z = position[2]

            # TODO: debug - show all checks that z is above terrain
            # if is_above:
            #     x = position[0]
            #     y = position[1]
            #     print(
            #         f"[{layer}]: "
            #         f"[{x:.2f}, {y:.2f}, {z:.2f}]; "
            #         f"ter_alt: {elevation:.2f}, agl_alt: {(z - elevation):.2f}"
            #     )

            if is_above:
                return elevation > z
            else:
                return elevation < z
        else:
            # Do not allow vehicle to go outside the map
            return True

    def setExclusionPolygons(self, polygons: list[list[tuple[float, float]]]) -> None:
        """
        Set one or more exclusion polygons

        :param polygons: A list of exclusion polygons (ENU frame)
        :type polygons: A list of polygons (each a point list)
        """
        self._exclusion_polygons.clear()
        for polygon in polygons:
            line = geometry.LineString(polygon)
            polygon = geometry.Polygon(line)
            self._exclusion_polygons.append(polygon)

    def setInclusionPolygons(self, polygons: list[list[tuple[float, float]]]) -> None:
        """
        Set one or more inclusion polygons

        :param polygons: A list of inclusion polygons (ENU frame)
        :type polygons: A list of polygons (poly = list[(east, north)])
        """
        self._inclusion_polygons.clear()
        for polygon in polygons:
            line = geometry.LineString(polygon)
            polygon = geometry.Polygon(line)
            self._inclusion_polygons.append(polygon)

    def setExclusionCircles(self, circles: list[tuple[float, float, float]]) -> None:
        """
        Set one or more exclusion circles

        :param circles: A list of exclusion circles (ENU frame)
        :type polygons: A list of circles (circle = (east, north, radius))
        """
        self._exclusion_circles = circles

    def setInclusionCircles(self, circles: list[tuple[float, float, float]]) -> None:
        """
        Set one or more exclusion circles

        :param circles: A list of exclusion circles (ENU frame)
        :type polygons: A list of circles (circle = (east, north, radius))
        """
        self._inclusion_circles = circles

    def isInsideExclusionPolygons(self, point: geometry.Point) -> bool:
        """
        Check if a point is inside or on the boundary of any exclusion polygon.

        :param point: The point to check if inside the polygons
        :return: `True` if the point is inside or on the boundary
        :rtype: bool
        """
        for polygon in self._exclusion_polygons:
            if polygon.contains(point) or polygon.boundary.contains(point):
                return True
        return False

    def isOutsideInclusionPolygons(self, point: geometry.Point) -> bool:
        """
        Check if a point is outside of all the inclusion polygons.

        :param point: The point to check if outside the polygons
        :return: `True` if the point is outside
        :rtype: bool
        """
        if not self._inclusion_polygons:
            return False

        for polygon in self._inclusion_polygons:
            if polygon.contains(point):
                return False
        return True

    def isInsideExclusionCircles(self, point: geometry.Point) -> bool:
        """
        Check if a point is inside or on the boundary of any exclusion circle.

        :param point: The point to check if inside the circle
        :return: `True` if the point is inside or on the boundary
        :rtype: bool
        """
        for x, y, radius in self._exclusion_circles:
            dx = point.x - x
            dy = point.y - y
            d2 = dx * dx + dy * dy
            r2 = radius * radius
            if d2 <= r2:
                return True
        return False

    def isOutsideInclusionCircles(self, point: geometry.Point) -> bool:
        """
        Check if a point is outside of all inclusion circle.

        :param point: The point to check if outside the circle
        :return: `True` if the point is outside
        :rtype: bool
        """
        if not self._inclusion_circles:
            return False

        for x, y, radius in self._inclusion_circles:
            dx = point.x - x
            dy = point.y - y
            d2 = dx * dx + dy * dy
            r2 = radius * radius
            if d2 <= r2:
                return False
        return True


class TerrainStateSampler(ob.StateSampler):
    def __init__(
        self,
        ss: ob.StateSpace,
        map: GridMap,
        max_altitude: float = 120.0,
        min_altitude: float = 50.0,
    ):
        """
        Initialise a terrain state sampler.

        :param ss: the DubinsAirplane state space
        :type ss: ob.StateSpace
        :param map: the terrain map
        :type map: GridMap
        :param max_altitude: maximum altitude to sample
        :type max_altitude: float
        :param min_altitude: minimum altitude to sample
        :type min_altitude: float
        """
        # TODO: test
        super().__init__(ss)
        self._rng = ou.RNG()
        self._map = map
        self._max_altitude: float = max_altitude
        self._min_altitude: float = min_altitude

    # TODO: We don't need to querry this everytime we sample
    def sampleUniform(self, state: ob.State) -> None:
        """
        Populate the state with uniform sample given terrain contraints.

        :param state: the state to modify
        :type state: ob.State
        """
        # TODO: test
        map_pos = self._map.getPosition()
        map_len = self._map.getLength()
        map_pos_x = map_pos[0]
        map_pos_y = map_pos[1]
        map_width_x = map_len[0]
        map_width_y = map_len[1]

        x = self._rng.uniformReal(
            map_pos_x - 0.5 * map_width_x, map_pos_x + 0.5 * map_width_x
        )
        y = self._rng.uniformReal(
            map_pos_y - 0.5 * map_width_y, map_pos_y + 0.5 * map_width_y
        )
        z = 0.0
        yaw = self._rng.uniformReal(-math.pi, math.pi)

        # TODO: Workaround when sampled position is not inside the map
        terrain_elevation = 0.0
        if self._map.isInside((x, y)):
            terrain_elevation = self._map.atPosition("elevation", (x, y))
        z = (
            self._rng.uniformReal(self._min_altitude, self._max_altitude)
            + terrain_elevation
        )

        # NOTE: use wrapper for access to internal state.
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        da_state.setXYZYaw(x, y, z, yaw)

    def sampleUniformNear(
        self, state: ob.State, near: ob.State, distance: float
    ) -> None:
        # NOTE: not implemented
        pass

    def sampleGaussian(self, state: ob.State, mean: ob.State, stdDev: float) -> None:
        # NOTE: not implemented
        pass
