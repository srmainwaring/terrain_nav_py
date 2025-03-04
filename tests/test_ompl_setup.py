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
import pytest

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace

from terrain_nav_py.ompl_setup import OmplSetup

from terrain_nav_py.ompl_setup import PlannerType
from terrain_nav_py.terrain_ompl import GridMap


def test_ompl_setup_planner_type():
    enum_value = PlannerType.RRTSTAR
    assert enum_value.value == 0
    assert str(enum_value) == "RRTSTAR"

    enum_value = PlannerType.INFORMED_RRTSTAR
    assert enum_value.value == 1
    assert str(enum_value) == "INFORMED_RRTSTAR"

    enum_value = PlannerType.RRTCONNECT
    assert enum_value.value == 2
    assert str(enum_value) == "RRTCONNECT"

    enum_value = PlannerType.BITSTAR
    assert enum_value.value == 3
    assert str(enum_value) == "BITSTAR"

    enum_value = PlannerType.FMTSTAR
    assert enum_value.value == 4
    assert str(enum_value) == "FMTSTAR"


def test_ompl_setup():
    # gridmap
    map = GridMap()

    # state space
    space = DubinsAirplaneStateSpace()

    # setup
    ompl_setup = OmplSetup(space)
    ompl_setup.setDefaultObjective()
    ompl_setup.setDefaultPlanner()
    ompl_setup.setStateValidityCheckingResolution(0.5)
    ompl_setup.setTerrainCollisionChecking(map, check_max_altitude=True)

    assert ompl_setup.getGeometricComponentStateSpace() == space
