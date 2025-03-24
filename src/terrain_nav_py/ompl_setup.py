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
Terrain planner OMPL setup
"""

from enum import IntEnum

from ompl import base as ob
from ompl import geometric as og

from terrain_nav_py.grid_map import GridMap

from terrain_nav_py.terrain_ompl import TerrainValidityChecker


class PlannerType(IntEnum):
    """
    Type of planner algorithm
    """

    RRTSTAR = 0
    INFORMED_RRTSTAR = 1
    RRTCONNECT = 2
    BITSTAR = 3
    FMTSTAR = 4

    def __str__(self):
        if self.value == PlannerType.RRTSTAR.value:
            return "RRTSTAR"
        elif self.value == PlannerType.INFORMED_RRTSTAR.value:
            return "INFORMED_RRTSTAR"
        elif self.value == PlannerType.RRTCONNECT.value:
            return "RRTCONNECT"
        elif self.value == PlannerType.BITSTAR.value:
            return "BITSTAR"
        elif self.value == PlannerType.FMTSTAR.value:
            return "FMTSTAR"
        else:
            return "INVALID"


class OmplSetup(og.SimpleSetup):
    """
    An extension of og.Setup for terrain navigation
    """

    # maximum length of a motion to be added in the tree of motions
    # DEFAULT_RANGE = 600.0
    DEFAULT_RANGE = 700.0

    def __init__(self, state_space):
        super().__init__(state_space)

    def setDefaultObjective(self) -> None:
        """
        Set the default objective
        """
        si = self.getSpaceInformation()
        oo = ob.PathLengthOptimizationObjective(si)
        self.setOptimizationObjective(oo)

    def setDefaultPlanner(
        self, planner_type: PlannerType = PlannerType.RRTSTAR
    ) -> None:
        """
        Set the default planner algorithm

        :param planner_type: the planner algorithm, defaults to PlannerType.RRTSTAR
        :type planner_type: PlannerType
        """
        if planner_type == PlannerType.RRTSTAR:
            planner = og.RRTstar(self.getSpaceInformation())
            planner.setRange(OmplSetup.DEFAULT_RANGE)
            self.setPlanner(planner)
        elif planner_type == PlannerType.RRTCONNECT:
            planner = og.RRTConnect(self.getSpaceInformation())
            planner.setRange(OmplSetup.DEFAULT_RANGE)
            self.setPlanner(planner)
        elif planner_type == PlannerType.INFORMED_RRTSTAR:
            planner = og.InformedRRTstar(self.getSpaceInformation())
            self.setPlanner(planner)
        elif planner_type == PlannerType.BITSTAR:
            planner = og.BITstar(self.getSpaceInformation())
            self.setPlanner(planner)
        elif planner_type == PlannerType.FMTSTAR:
            planner = og.FMT(self.getSpaceInformation())
            self.setPlanner(planner)

    def getGeometricComponentStateSpace(self) -> ob.StateSpace:
        """
        Get the state space
        """
        return self.getStateSpace()

    def setStateValidityCheckingResolution(self, resolution: float) -> None:
        """
        Set the validity checker resolution

        :param resolution: checker resolution in range [0, 1]
        :type resolution: float
        """
        self.getSpaceInformation().setStateValidityCheckingResolution(resolution)

    def setTerrainCollisionChecking(
        self, map: GridMap, check_max_altitude: bool
    ) -> None:
        """
        Set a terrain collision validity checker

        :param map: the terrain map
        :type map: GridMap
        :param check_max_altitude: set true to check maximum altitude
        :type check_max_altitude: bool
        """
        validity_checker = TerrainValidityChecker(
            self.getSpaceInformation(), map, check_max_altitude
        )
        self.setStateValidityChecker(validity_checker)

    def setExclusionPolygons(
        self, polygons: list[list[tuple[float, float]]]
    ) -> None:
        validity_checker = self.getStateValidityChecker()
        validity_checker.setExclusionPolygons(polygons)

    def setInclusionPolygons(
        self, polygons: list[list[tuple[float, float]]]
    ) -> None:
        validity_checker = self.getStateValidityChecker()
        validity_checker.setInclusionPolygons(polygons)

    def setExclusionCircles(
        self, circles: list[tuple[float, float, float]]
    ) -> None:
        validity_checker = self.getStateValidityChecker()
        validity_checker.setExclusionCircles(circles)

    def setInclusionCircles(
        self, circles: list[tuple[float, float, float]]
    ) -> None:
        validity_checker = self.getStateValidityChecker()
        validity_checker.setInclusionCircles(circles)
