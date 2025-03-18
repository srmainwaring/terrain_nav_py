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

"""
GridMap
"""

import copy

import numpy as np

from MAVProxy.modules.lib import mp_elevation
from MAVProxy.modules.lib import mp_util


class GridMap:
    """
    Interface for GridMap
    """

    class Index:
        def __init__(self):
            self.value = 0

    class Iterator:
        def __init__(self, map):
            self._start = GridMap.Index()
            self._index = GridMap.Index()
            self._size = map.size()
            self._is_past_end = False

        def __iter__(self):
            return self

        def __next__(self):
            current = copy.deepcopy(self._index)

            if self._index.value == self._size:
                self._is_past_end = True

            if not self._is_past_end:
                self._index.value += 1
            else:
                raise StopIteration

            return current

    def __init__(self):
        self._position = (0.0, 0.0)
        self._length = (1000.0, 1000.0)
        self._elevation = 0.0
        self._max_elevation = 120.0
        self._min_elevation = 50.0
        self._size = 10

    def __iter__(self) -> Iterator:
        return GridMap.Iterator(self)

    def size(self) -> int:
        return self._size

    def getPosition(self) -> tuple[float, float]:
        """
        The 2d position of the grid map in the grid map frame.
        """
        return self._position

    def getLength(self) -> tuple[float, float]:
        """
        The length (2d) of the grid map sides.
        """
        return self._length

    def isInside(self, position: tuple[float, float]) -> bool:
        """
        Check if a 2d position is inside the grid map boundary.
        """
        return True

    def atPosition(self, layer: str, position: tuple[float, float]) -> float:
        """
        Return the layer value at the given cartesian position.
        """
        if layer == "elevation":
            return self._elevation
        elif layer == "distance_surface":
            return self._min_elevation
        elif layer == "max_elevation":
            return self._max_elevation
        else:
            return float("nan")

    def at(self, layer: str, index: Index) -> float:
        """
        Return the layer value at the given index.
        """
        if layer == "elevation":
            return self._elevation
        elif layer == "distance_surface":
            return self._min_elevation
        elif layer == "max_elevation":
            return self._max_elevation
        else:
            return float("nan")


class GridMapSRTM(GridMap):
    """
    A GridMap using SRTM terrain data accessed using MAVProxy tools.

    USGS EROS Archive - Digital Elevation - Shuttle Radar Topography Mission (SRTM) 1 Arc-Second Global
    https://www.usgs.gov/centers/eros/science/usgs-eros-archive-digital-elevation-shuttle-radar-topography-mission-srtm-1

    Projection	Geographic
    Horizontal Datum	WGS84
    Vertical Datum	EGM96 (Earth Gravitational Model 1996)
    Vertical Units	Meters
    Spatial Resolution	1 arc-second for global coverage (~30 meters)
                        3 arc-seconds for global coverage (~90 meters)
    Raster Size	1 degree tiles


    NOTE: the planner follows ROS conventions and must use ENU coordinates.
    """

    def __init__(self, map_lat, map_lon, max_elevation=120.0, min_elevation=50.0):
        super().__init__()

        self._map_lat = map_lat
        self._map_lon = map_lon
        self._grid_spacing = 30
        self._grid_length = 10000
        self._terrain_source = "SRTM1"
        self._terrain_offline = False
        self._terrain_timeout = 10.0
        self._elevation_model = mp_elevation.ElevationModel(
            database=self._terrain_source, offline=self._terrain_offline
        )

        # precalculated grid x: east, y: north
        self._x = np.array([])
        self._y = np.array([])

        # set super class properties
        self._position = (0.0, 0.0)
        self._max_elevation = max_elevation
        self._min_elevation = min_elevation

        # ensure grid is updated
        self._updateGrid()

    def _updateGrid(self) -> None:
        """
        Internal method to update the precalculated grid positions
        """
        # update precalculated grid positions
        self._x = np.arange(
            -0.5 * self._grid_length, 0.5 * self._grid_length, self._grid_spacing
        )
        self._y = np.arange(
            -0.5 * self._grid_length, 0.5 * self._grid_length, self._grid_spacing
        )

        # update super class properties
        self._size = len(self._x) * len(self._y)
        self._length = (self._grid_length, self._grid_length)

    def setGridSpacing(self, value: float) -> None:
        """
        Set the grid spacing (m)
        """
        self._grid_spacing = value
        self._updateGrid()

    def setGridLength(self, value: float) -> None:
        """
        Set the grid length (m)
        """
        self._grid_length = value
        self._updateGrid()

    def isInside(self, position: tuple[float, float]) -> bool:
        """
        Check if a 2d position is inside the grid map boundary.
        """
        x = position[0]
        y = position[1]
        is_valid = (
            -0.5 * self._grid_length <= x and x <= 0.5 * self._grid_length
        ) and (-0.5 * self._grid_length <= y and y <= 0.5 * self._grid_length)
        return is_valid

    def atPosition(self, layer: str, position: tuple[float, float]) -> float:
        """
        Return the layer value at the given cartesian position.
        """
        east = position[0]
        north = position[1]
        (lat2, lon2) = (lat2, lon2) = mp_util.gps_offset(
            self._map_lat, self._map_lon, east, north
        )
        alt = self._elevation_model.GetElevation(lat2, lon2, self._terrain_timeout)
        if alt is None:
            raise ValueError(
                f"[GridMapSRTM] invalid elevation, "
                f"failed to load terrain data for {self._terrain_source} "
                f"at lat: {lat2}, lon: {lon2}"
            )

        if layer == "elevation":
            return alt
        elif layer == "distance_surface":
            return alt + self._min_elevation
        elif layer == "max_elevation":
            return alt + self._max_elevation
        else:
            return float("nan")

    def at(self, layer: str, index: GridMap.Index) -> float:
        """
        Return the layer value at the given index.
        """
        size_y = len(self._y)
        i_x = index.value // size_y
        i_y = index.value % size_y
        x = self._x[i_x]
        y = self._y[i_y]
        return self.atPosition(layer, (x, y))
    

    def addLayerDistanceTransform(self):
        surface_distance = 50
        reference_layer = "elevation"

        for x in self._x:
            for y in self._y:
                position_2d = (x, y)
                elevation = self.atPosition(reference_layer, position_2d)
