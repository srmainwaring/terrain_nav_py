"""
Terrain planner OMPL
"""

# Original C++ version
#
# terrain_ompl.h

import math
import sys

from ompl import base as ob
from ompl import util as ou

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace


# Mock interface for GridMap
class GridMap:
    def __init__(self):
        self._position = (0.0, 0.0)
        self._length = (1000.0, 1000.0)
        self._elevation = 0.0
        self._distance_surface = 0.0
        self._max_elevation = 120.0

    def getPosition(self) -> tuple[float, float]:
        return self._position

    def getLength(self) -> tuple[float, float]:
        return self._length

    def isInside(self, position: tuple[float, float]) -> bool:
        return True

    def atPosition(self, layer: str, position: tuple[float, float]) -> float:
        if layer == "elevation":
            return self._elevation
        elif layer == "distance_surface":
            return self._distance_surface
        elif layer == "max_elevation":
            return self._max_elevation
        else:
            return float("nan")


class TerrainValidityChecker(ob.StateValidityChecker):
    def __init__(
        self, space_info: ob.SpaceInformation, map: GridMap, check_max_altitude: bool
    ):
        super().__init__(space_info)
        self._map = map
        self._check_collision_max_altitude = check_max_altitude

    def isValid(self, state: ob.State) -> bool:
        """
        State validity check

        :param state: the state to check
        :type state: ob.State
        """
        # TODO test

        # NOTE: use wrapper for access to internal state.
        da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)
        position = (da_state.getX(), da_state.getY(), da_state.getZ())

        # DEBUG
        # print(f"position: {position}")
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
        Check if the state is in collision with a layer

        :param layer: name of the layer in the elevation map
        :param position: position of the state to evaluate
        :param is_above:
        """
        # TODO test
        position_2d = (position[0], position[1])
        if self._map.isInside(position_2d):
            elevation = self._map.atPosition(layer, position_2d)
            z = position[2]
            if is_above:
                return elevation > z
            else:
                return elevation < z
        else:
            # Do not allow vehicle to go outside the map
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
        # DEBUG:
        # print(f"{x}, {y}, {z}, {yaw}")
        # print(f"{state()[0][0]}, {state()[0][1]}, {state()[0][2]}, {state()[1].value}")

    def sampleUniformNear(
        self, state: ob.State, near: ob.State, distance: float
    ) -> None:
        # TODO: test
        # print("Sample Near")
        pass

    def sampleGaussian(self, state: ob.State, mean: ob.State, stdDev: float) -> None:
        # TODO: test
        # print("Sample Gaussian")
        pass
