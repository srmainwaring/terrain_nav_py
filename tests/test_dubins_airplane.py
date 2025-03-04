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

from ompl import base as ob
from ompl import geometric as og


from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace

def test_ompl_base_states():

    # R(3) state space
    r3_ss = ob.RealVectorStateSpace(3)
    # State
    r3_s = ob.State(r3_ss)
    # RealVectorStateInternal
    r3_s_ref = r3_s()
    # accessors
    assert r3_s_ref[0] == 0.0
    r3_s_ref[0] = 1.0
    assert r3_s_ref[0] == 1.0

    # SO(2) state space
    so2_ss = ob.SO2StateSpace()
    # State
    so2_s = ob.State(so2_ss)
    # SO2StateInternal
    so2_s_ref = so2_s()
    # accessors
    assert so2_s_ref.value == 0.0
    so2_s_ref.value = 1.0
    assert so2_s_ref.value == 1.0

    # compound state space
    c_ss = ob.CompoundStateSpace()
    c_ss.addSubspace(r3_ss, 1.0)
    c_ss.addSubspace(so2_ss, 1.0)
    c_ss.lock()
    # State
    c_s = ob.State(c_ss)
    # CompoundStateInternal
    c_s_ref = c_s()

    # state type conversions
    r_s = c_s_ref[0]
    s_s = c_s_ref[1]

    assert r_s[0] == 0.0
    assert s_s.value == 0.0


def test_dubins_airplane_state():
    # compound state space
    c_ss = ob.CompoundStateSpace()
    c_ss.addSubspace(ob.RealVectorStateSpace(3), 1.0)
    c_ss.addSubspace(ob.SO2StateSpace(), 1.0)
    c_ss.lock()

    s = DubinsAirplaneStateSpace.DubinsAirplaneState(ob.State(c_ss))
    assert s.getX() == 0.0
    assert s.getY() == 0.0
    assert s.getZ() == 0.0
    assert s.getYaw() == 0.0

    s.setX(100.0)
    s.setY(200.0)
    s.setZ(50.0)
    s.setYaw(0.25 * math.pi)
    assert s.getX() == 100.0
    assert s.getY() == 200.0
    assert s.getZ() == 50.0
    assert s.getYaw() == 0.25 * math.pi

    s.setXYZ(120.0, 210.0, 60.0)
    assert s.getX() == 120.0
    assert s.getY() == 210.0
    assert s.getZ() == 60.0

    s.setXYZYaw(150.0, 230.0, 45.0, 0.5 * math.pi)
    assert s.getX() == 150.0
    assert s.getY() == 230.0
    assert s.getZ() == 45.0
    assert s.getYaw() == 0.5 * math.pi

    s.addToX(30.0)
    assert s.getX() == 180.0

    s.addToY(35.0)
    assert s.getY() == 265.0

    s.addToZ(-5.0)
    assert s.getZ() == 40.0


def test_dubins_airplane_state_space():
    ss = DubinsAirplaneStateSpace()
    # TODO: why DubinsAirplaneCompoundSpace6?
    # assert ss.getName() == "DubinsAirplaneCompoundSpace3"

    # TODO: fix type
    assert ss.getType() == ob.StateSpaceType.STATE_SPACE_UNKNOWN
    # assert ss.getType() == ob.StateSpaceType.STATE_SPACE_SE3

    # max extent is pi for null R(3) x SO(2)
    e = ss.getMaximumExtent()
    assert e == math.pi

    e = ss.getEuclideanExtent()
    assert e == 0.0

    # state1 = ob.State()
    # state2 = ob.State()
    # count = ss.validSegmentCount(state1, state2)
    # dist = ss.distance(state1, state2)

def test_dubins_airplane_alloc_state():

    # state space
    space = DubinsAirplaneStateSpace()

    ss = og.SimpleSetup(space)
    si = ss.getSpaceInformation()

    # cs = ob.CompoundState(si)
    # print(type(cs))

