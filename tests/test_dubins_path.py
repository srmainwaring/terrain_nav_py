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

from terrain_nav_py.dubins_path import DubinsPath


def test_dubins_path_segment_type():
    enum_value = DubinsPath.DubinsPathSegmentType.DUBINS_LEFT
    assert enum_value.value == 0
    assert str(enum_value) == "DUBINS_LEFT"

    enum_value = DubinsPath.DubinsPathSegmentType.DUBINS_STRAIGHT
    assert enum_value.value == 1
    assert str(enum_value) == "DUBINS_STRAIGHT"

    enum_value = DubinsPath.DubinsPathSegmentType.DUBINS_RIGHT
    assert enum_value.value == 2
    assert str(enum_value) == "DUBINS_RIGHT"


def test_dubins_path_classification():
    enum_value = DubinsPath.Classification.CLASS_A11
    assert enum_value.value == 0
    assert str(enum_value) == "CLASS_A11"

    enum_value = DubinsPath.Classification.CLASS_A12
    assert enum_value.value == 1
    assert str(enum_value) == "CLASS_A12"

    enum_value = DubinsPath.Classification.CLASS_A13
    assert enum_value.value == 2
    assert str(enum_value) == "CLASS_A13"

    enum_value = DubinsPath.Classification.CLASS_A14
    assert enum_value.value == 3
    assert str(enum_value) == "CLASS_A14"

    enum_value = DubinsPath.Classification.CLASS_A41
    assert enum_value.value == 12
    assert str(enum_value) == "CLASS_A41"

    enum_value = DubinsPath.Classification.CLASS_A42
    assert enum_value.value == 13
    assert str(enum_value) == "CLASS_A42"

    enum_value = DubinsPath.Classification.CLASS_A43
    assert enum_value.value == 14
    assert str(enum_value) == "CLASS_A43"

    enum_value = DubinsPath.Classification.CLASS_A44
    assert enum_value.value == 15
    assert str(enum_value) == "CLASS_A44"


def test_dubins_path_index():
    enum_value = DubinsPath.Index.TYPE_LSL
    assert enum_value.value == 0
    assert str(enum_value) == "TYPE_LSL"

    enum_value = DubinsPath.Index.TYPE_RSR
    assert enum_value.value == 1
    assert str(enum_value) == "TYPE_RSR"

    enum_value = DubinsPath.Index.TYPE_RSL
    assert enum_value.value == 2
    assert str(enum_value) == "TYPE_RSL"

    enum_value = DubinsPath.Index.TYPE_LSR
    assert enum_value.value == 3
    assert str(enum_value) == "TYPE_LSR"

    enum_value = DubinsPath.Index.TYPE_RLR
    assert enum_value.value == 4
    assert str(enum_value) == "TYPE_RLR"

    enum_value = DubinsPath.Index.TYPE_LRL
    assert enum_value.value == 5
    assert str(enum_value) == "TYPE_LRL"


def test_dubins_path_attitude_case():
    enum_value = DubinsPath.AltitudeCase.ALT_CASE_LOW
    assert enum_value.value == 0
    assert str(enum_value) == "ALT_CASE_LOW"

    enum_value = DubinsPath.AltitudeCase.ALT_CASE_MEDIUM
    assert enum_value.value == 1
    assert str(enum_value) == "ALT_CASE_MEDIUM"

    enum_value = DubinsPath.AltitudeCase.ALT_CASE_HIGH
    assert enum_value.value == 2
    assert str(enum_value) == "ALT_CASE_HIGH"


def test_dubins_path_init():
    # default
    path = DubinsPath()

    seg_type = DubinsPath.Index.TYPE_LRL
    # t = 0.0
    # p = float("nan")
    # q = 0.0
    gam = 0.0
    # ks = 0
    # ke = 0
    r = 1.0

    assert math.isnan(path.length_2d())
    assert math.isnan(path.length_3d())
    assert path.getFoundOptimalPath() == True
    assert path.getAdditionalManeuver() == False
    assert path.getAltitudeCase() == DubinsPath.AltitudeCase.ALT_CASE_LOW
    assert path.getIdx() == seg_type
    assert path.getClassification() == DubinsPath.Classification.NOT_ASSIGNED
    assert path.getGamma() == gam
    assert path.getRadiusRatio(0) == r
    assert path.getInverseRadiusRatio(0) == 1.0 / r
    assert path.getSegmentLength(0) == 0.0
    assert path.getType() == DubinsPath.dubinsPathType[seg_type]
