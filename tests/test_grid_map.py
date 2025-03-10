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
    home_lat = 56.6987387
    home_lon = -6.1082210
    home_alt = 2.74

    # Add terrain contours to map #1508
    # https://github.com/ArduPilot/MAVProxy/pull/1508

    # load mavproxy elevation model
    terrain_source = "SRTM1"
    terrain_offline = False
    elevation_model = mp_elevation.ElevationModel(
        database=terrain_source, offline=terrain_offline
    )

    # create a mesh grid (m)
    grid_spacing = 30
    grid_extent = 10000
    x = np.arange(-0.5 * grid_extent, 0.5 * grid_extent, grid_spacing)
    y = np.arange(-0.5 * grid_extent, 0.5 * grid_extent, grid_spacing)
    x_grid, y_grid = np.meshgrid(x, y)

    def terrain_surface(lat, lon, x, y):
        """
        Calculate terrain altitudes for the ENU offsets (x, y)
        centred on (lat, lon).
        """
        alt = []
        for north in y:
            alt_y = []
            for east in x:
                (lat2, lon2) = mp_util.gps_offset(lat, lon, east, north)
                alt_y.append(elevation_model.GetElevation(lat2, lon2))
            alt.append(alt_y)
        return alt

    # generate surface
    z_grid = np.array(terrain_surface(home_lat, home_lon, x, y))
    # print(z_grid)


def test_grid_map_srtm():
    # Kilchoan
    # TERRAIN_REPORT {lat : 566987387, lon : -61082210, spacing : 0, terrain_height : 0.0, current_height : 2.809999942779541, pending : 56, loaded : 336}
    # --custom-location="56.6987387,-6.1082210,2.74,0.0"
    # orthometric coordinates (AMSL)
    home_lat = 56.6987387
    home_lon = -6.1082210
    home_alt = 2.74

    # Add terrain contours to map #1508
    # https://github.com/ArduPilot/MAVProxy/pull/1508

    # load mavproxy elevation model
    terrain_source = "SRTM1"
    terrain_offline = False
    elevation_model = mp_elevation.ElevationModel(
        database=terrain_source, offline=terrain_offline
    )

    grid_map = GridMapSRTM(home_lat, home_lon)

    # check default grid extents
    assert grid_map.getLength()[0] == 10000
    assert grid_map.getLength()[1] == 10000

    # terrain at a position (ENU)
    position = (200, 100)
    (lat, lon) = mp_util.gps_offset(
        home_lat, home_lon, east=position[0], north=position[1]
    )
    expected_alt = elevation_model.GetElevation(lat, lon)
    assert expected_alt != 0.0

    alt = grid_map.atPosition("elevation", position)
    assert alt == expected_alt

    alt = grid_map.atPosition("distance_surface", position)
    assert alt == expected_alt + 0.0

    alt = grid_map.atPosition("max_elevation", position)
    assert alt == expected_alt + 120.0

    # terrain at index (use to find max and min over grid)
    assert grid_map.size() == (334) * (334)

    alt_min = 10000.0
    alt_max = -10000.0
    for idx in grid_map:
        alt = grid_map.at("elevation", idx)
        if alt < alt_min:
            alt_min = alt
        if alt > alt_max:
            alt_max = alt

    print(f"alt_min: {alt_min:.2f}, alt_max: {alt_max:.2f}")


def main():
    test_grid_map_srtm()


if __name__ == "__main__":
    main()
