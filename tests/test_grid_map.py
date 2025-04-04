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

import numpy as np
import random

from MAVProxy.modules.lib import mp_elevation
from MAVProxy.modules.lib import mp_util

from terrain_nav_py.grid_map import GridMapSRTM


def test_load_mavproxy_elevation_module():
    """
    Check that the MAVProxy elevation module can be loaded.
    """
    # Kilchoan
    # TERRAIN_REPORT {lat : 566987387, lon : -61082210, spacing : 0, terrain_height : 0.0, current_height : 2.809999942779541, pending : 56, loaded : 336}
    # --custom-location="56.6987387,-6.1082210,2.74,0.0"
    # orthometric coordinates (AMSL)
    map_lat = 56.6987387
    map_lon = -6.1082210

    # load mavproxy elevation model
    terrain_source = "SRTM1"
    terrain_offline = False
    terrain_timeout = 10.0
    elevation_model = mp_elevation.ElevationModel(
        database=terrain_source, offline=terrain_offline
    )

    # create a mesh grid (m)
    grid_spacing = 30
    grid_extent = 10000
    num_step = int(grid_extent / grid_spacing)
    x = np.linspace(-0.5 * grid_extent, 0.5 * grid_extent, num_step)
    y = np.linspace(-0.5 * grid_extent, 0.5 * grid_extent, num_step)
    x_grid, y_grid = np.meshgrid(x, y)

    def terrain_surface(lat, lon, x, y):
        """
        Calculate terrain altitudes for the ENU offsets (x, y)
        centred on (lat, lon).
        """
        alt = []
        for east in x:
            alt_x = []
            for north in y:
                (lat2, lon2) = mp_util.gps_offset(lat, lon, east, north)
                alt_x.append(elevation_model.GetElevation(lat2, lon2, terrain_timeout))
            alt.append(alt_x)
        return alt

    # generate surface
    z_grid = np.array(terrain_surface(map_lat, map_lon, x, y))
    # print(z_grid)


def test_grid_map_srtm():
    # Kilchoan
    # TERRAIN_REPORT {lat : 566987387, lon : -61082210, spacing : 0, terrain_height : 0.0, current_height : 2.809999942779541, pending : 56, loaded : 336}
    # --custom-location="56.6987387,-6.1082210,2.74,0.0"
    # orthometric coordinates (AMSL)
    map_lat = 56.6987387
    map_lon = -6.1082210

    # load mavproxy elevation model
    terrain_source = "SRTM1"
    terrain_offline = False
    terrain_timeout = 10.0
    elevation_model = mp_elevation.ElevationModel(
        database=terrain_source, offline=terrain_offline
    )

    max_altitude = 120.0
    min_altitude = 50.0
    grid_map = GridMapSRTM(
        map_lat, map_lon, max_elevation=max_altitude, min_elevation=min_altitude
    )

    # check default grid extents
    assert grid_map.getLength()[0] == 10000
    assert grid_map.getLength()[1] == 10000

    # terrain at a position (ENU)
    position = (200, 100)
    (lat, lon) = mp_util.gps_offset(
        map_lat, map_lon, east=position[0], north=position[1]
    )
    expected_alt = elevation_model.GetElevation(lat, lon, terrain_timeout)
    assert expected_alt != 0.0

    alt = grid_map.atPosition("elevation", position)
    assert alt == expected_alt

    alt = grid_map.atPosition("distance_surface", position)
    assert alt == expected_alt + min_altitude

    alt = grid_map.atPosition("max_elevation", position)
    assert alt == expected_alt + max_altitude

    # terrain at index (use to find max and min over grid)
    assert grid_map.size() == (333) * (333)

    alt_min = 10000.0
    alt_max = -10000.0
    for idx in grid_map:
        alt = grid_map.at("elevation", idx)
        if alt < alt_min:
            alt_min = alt
        if alt > alt_max:
            alt_max = alt

    print(f"alt_min: {alt_min:.2f}, alt_max: {alt_max:.2f}")


def test_grid_map_srtm_loading():
    """
    Test terrain tiles are loading within timeout=10.0.
    """
    terrain_source = "SRTM1"
    terrain_offline = False
    terrain_timeout = 10.0

    elevation_model = mp_elevation.ElevationModel(
        database=terrain_source, offline=terrain_offline
    )

    for i in range(10):
        lat = random.uniform(-60.0, 60.0)
        lon = random.uniform(-180.0, 180.0)
        alt = elevation_model.GetElevation(lat, lon, terrain_timeout)
        assert alt is not None


def test_grid_map_distance_layer():
    # Kilchoan
    # TERRAIN_REPORT {lat : 566987387, lon : -61082210, spacing : 0, terrain_height : 0.0, current_height : 2.809999942779541, pending : 56, loaded : 336}
    # --custom-location="56.6987387,-6.1082210,2.74,0.0"
    # orthometric coordinates (AMSL)
    map_lat = 56.6987387
    map_lon = -6.1082210

    # Davos
    map_lat = 46.8201124
    map_lon = 9.8260916

    # load mavproxy elevation model
    terrain_source = "SRTM1"
    terrain_offline = False
    # terrain_timeout = 10.0
    # elevation_model = mp_elevation.ElevationModel(
    #     database=terrain_source, offline=terrain_offline
    # )

    max_altitude = 120.0
    min_altitude = 50.0
    grid_length = 300.0
    grid_spacing = 30.0

    grid_map = GridMapSRTM(
        map_lat, map_lon, max_elevation=max_altitude, min_elevation=min_altitude
    )
    grid_map.setGridSpacing(grid_spacing)
    grid_map.setGridLength(grid_length)

    grid_map.addLayerDistanceTransform(surface_distance=min_altitude)

    # check that the bounds are inclusive
    # print(grid_map._x[0], grid_map._x[-1])
    # print(grid_map._y[0], grid_map._y[-1])

    # random check that distance surface is above elevation
    min_dz = max_altitude
    max_dz = 0.0
    for i in range(1000):
        x = random.uniform(-0.5 * 300.0, 0.5 * 300.0)
        y = random.uniform(-0.5 * 300.0, 0.5 * 300.0)
        position = (x, y)
        elev_alt = grid_map.atPosition("elevation", position)
        surf_alt = grid_map.atPosition("distance_surface", position)
        # print(f"elev_alt: {elev_alt}")
        # print(f"surf_alt: {surf_alt}")
        # print(f"dz: {surf_alt - elev_alt}")
        assert (surf_alt - elev_alt) >= 0.0
        # assert (surf_alt - elev_alt) >= min_altitude
        min_dz = min(min_dz, surf_alt - elev_alt)
        max_dz = max(max_dz, surf_alt - elev_alt)

    print(f"min_dz: {min_dz:.1f}, max_dz: {max_dz:.1f}")

def test_grid_map_subgrid():
    # locate grid points within a circle of radius at centre_pos
    radius = 20
    centre_pos = np.array([20, -10])

    # create a grid with length and spacing
    spacing = 10
    length = 100
    num_step = int(length / spacing)
    x = np.linspace(-0.5 * length, 0.5 * length, num_step)
    y = np.linspace(-0.5 * length, 0.5 * length, num_step)
    x_grid, y_grid = np.meshgrid(x, y)

    def circleSlice(x_grid, y_grid, centre_pos, radius):
        # set up selection conditions 
        r2 = radius * radius
        dx = centre_pos[0] - x_grid
        dy = centre_pos[1] - y_grid
        d2 = dx * dx + dy * dy
        d1 = np.round(np.sqrt(d2), 2)

        # boolean array, the flattened
        is_inside = d2 <= r2
        is_inside_flat = is_inside.reshape(is_inside.size)

        # create then flatten indices
        idx_xy = np.indices(is_inside.shape, dtype=int)
        idx_x = idx_xy[0].reshape((idx_xy[0].size))
        idx_y = idx_xy[1].reshape((idx_xy[1].size))

        # TODO: check index convention
        # boolean slice, then zip to form array of 2d indices
        sub_idx = list(zip(idx_x[is_inside_flat], idx_y[is_inside_flat]))
        return (sub_idx, d1)

    (indices, d1) = circleSlice(x_grid, y_grid, centre_pos, radius)

    for idx in indices:
        (i, j) = idx
        z = d1[i][j]
        print(f"d1[{i}][{j}] = {z}")


def main():
    test_grid_map_distance_layer()


if __name__ == "__main__":
    main()
