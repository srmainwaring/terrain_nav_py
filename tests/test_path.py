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

from pymavlink import quaternion
from pymavlink.rotmat import Matrix3
from pymavlink.rotmat import Vector3

from terrain_nav_py.path_segment import State
from terrain_nav_py.path_segment import PathSegment
from terrain_nav_py.path import Path

FLOAT_EPS = 1.0e-6


def test_path():
    # air speed (m/s)
    speed = 15.0
    alt = 50.0
    radius = 50.0
    kappa = 1.0 / radius

    # create states
    m_att1 = Matrix3()
    m_att1.from_euler(0.0, 0.0, -90.0)
    state1 = State()
    state1.position = Vector3(50.0, 100.0, alt)
    state1.velocity = Vector3(0.0, -1.0 * speed, 0.0)
    state1.attitude = quaternion.Quaternion(m_att1)

    m_att2 = Matrix3()
    m_att2.from_euler(0.0, 0.0, 0.0)
    state2 = State()
    state2.position = Vector3(100.0, 50.0, alt)
    state2.velocity = Vector3(1.0 * speed, 0.0, 0.0)
    state2.attitude = quaternion.Quaternion(m_att2)

    m_att3 = Matrix3()
    m_att3.from_euler(0.0, 0.0, 0.0)
    state3 = State()
    state3.position = Vector3(150.0, 50.0, alt)
    state3.velocity = Vector3(1.0 * speed, 0.0, 0.0)
    state3.attitude = quaternion.Quaternion(m_att3)

    m_att4 = Matrix3()
    m_att4.from_euler(0.0, 0.0, -90.0)
    state4 = State()
    state4.position = Vector3(200.0, 0.0, alt)
    state4.velocity = Vector3(0.0, -1.0 * speed, 0.0)
    state4.attitude = quaternion.Quaternion(m_att4)

    # half-turn ccw
    path_segment1 = PathSegment()
    path_segment1.curvature = kappa
    path_segment1.is_periodic = False
    path_segment1.reached = False
    path_segment1.append_state(state1)
    path_segment1.append_state(state2)

    # line parallel to x-axis
    path_segment2 = PathSegment()
    path_segment2.curvature = 0.0
    path_segment2.is_periodic = False
    path_segment2.reached = False
    path_segment2.append_state(state2)
    path_segment2.append_state(state3)

    # half-turn cw
    path_segment3 = PathSegment()
    path_segment3.curvature = -1.0 * kappa
    path_segment3.is_periodic = False
    path_segment3.reached = False
    path_segment3.append_state(state3)
    path_segment3.append_state(state4)

    # create path comprising 3 segments
    path = Path()
    path.append_segment(path_segment1)
    path.append_segment(path_segment2)
    path.append_segment(path_segment3)

    assert path.first_segment() == path_segment1
    assert path.last_segment() == path_segment3


def test_path_get_closest_point1():
    trajectory = Path()
    first_arc_segment = PathSegment()

    segment_start = State()
    segment_start.position = Vector3(0.0, 0.0, 0.0)
    segment_start.velocity = Vector3(1.0, 0.0, 0.0)

    segment_middle = State()
    segment_middle.position = Vector3(1.0, 1.0, 0.0)
    segment_middle.velocity = Vector3(0.0, 1.0, 0.0)

    first_arc_segment.curvature = 1.0
    first_arc_segment.append_state(segment_start)
    first_arc_segment.append_state(segment_middle)
    trajectory.append_segment(first_arc_segment)

    second_straight_segment = PathSegment()

    second_segment_end = State()
    second_segment_end.position = Vector3(1.0, 2.0, 0.0)
    second_segment_end.velocity = Vector3(0.0, 1.0, 0.0)

    second_straight_segment.curvature = 0.0
    second_straight_segment.append_state(segment_middle)
    second_straight_segment.append_state(second_segment_end)
    trajectory.append_segment(second_straight_segment)

    query_poistion = Vector3()
    closest_point = Vector3()
    tangent = Vector3()
    curvature = 0.0

    query_poistion = Vector3(0.0, -1.0, 0.0)
    (closest_point, tangent, curvature) = trajectory.get_closest_point(query_poistion)
    assert closest_point.x == pytest.approx(Vector3().x)
    assert closest_point.y == pytest.approx(Vector3().y)
    assert closest_point.z == pytest.approx(Vector3().z)
    assert tangent.x == pytest.approx(Vector3(1.0, 0.0, 0.0).x)
    assert tangent.y == pytest.approx(Vector3(1.0, 0.0, 0.0).y)
    assert tangent.z == pytest.approx(Vector3(1.0, 0.0, 0.0).z)
    assert curvature == 1.0

    query_poistion = Vector3(1.0, 0.0, 0.0)
    (closest_point, tangent, curvature) = trajectory.get_closest_point(query_poistion)
    expected_closest_point = Vector3(
        math.sin(math.pi / 4.0), 1.0 - math.cos(math.pi / 4.0), 0.0
    )
    assert closest_point.x == pytest.approx(expected_closest_point.x)
    assert closest_point.y == pytest.approx(expected_closest_point.y)
    assert closest_point.z == pytest.approx(expected_closest_point.z)

    expected_closest_point = Vector3(
        math.sin(math.pi / 4.0), 1.0 - math.cos(math.pi / 4.0), 0.0
    )
    assert (closest_point - expected_closest_point).length() < FLOAT_EPS
    expected_tangent = Vector3(1.0, 1.0, 0.0).normalized()
    assert tangent.x == pytest.approx(expected_tangent.x)
    assert tangent.y == pytest.approx(expected_tangent.y)
    assert tangent.z == pytest.approx(expected_tangent.z)
    assert curvature == 1.0

    query_poistion = Vector3(2.0, 1.1, 0.0)
    (closest_point, tangent, curvature) = trajectory.get_closest_point(query_poistion)
    assert closest_point == Vector3(1.0, 1.1, 0.0)
    assert tangent == Vector3(0.0, 1.0, 0.0)
    assert curvature == 0.0


def test_path_get_closest_point2():
    trajectory = Path()
    first_arc_segment = PathSegment()

    segment_start = State()
    segment_start.position = Vector3(0.0, 0.0, 0.0)
    segment_start.velocity = Vector3(1.0, 0.0, 0.0)

    segment_middle = State()
    segment_middle.position = Vector3(1.0, -1.0, 0.0)
    segment_middle.velocity = Vector3(0.0, -1.0, 0.0)

    first_arc_segment.curvature = -1.0
    first_arc_segment.append_state(segment_start)
    first_arc_segment.append_state(segment_middle)
    trajectory.append_segment(first_arc_segment)

    second_straight_segment = PathSegment()

    second_segment_end = State()
    second_segment_end.position = Vector3(1.0, -2.0, 0.0)
    second_segment_end.velocity = Vector3(0.0, -1.0, 0.0)

    second_straight_segment.curvature = 0.0
    second_straight_segment.append_state(segment_middle)
    second_straight_segment.append_state(second_segment_end)
    trajectory.append_segment(second_straight_segment)

    query_poistion = Vector3()
    closest_point = Vector3()
    tangent = Vector3()
    curvature = 0.0

    query_poistion = Vector3(0.0, 1.0, 0.0)
    (closest_point, tangent, curvature) = trajectory.get_closest_point(query_poistion)
    assert closest_point.x == pytest.approx(Vector3().x)
    assert closest_point.y == pytest.approx(Vector3().y)
    assert closest_point.z == pytest.approx(Vector3().z)
    assert tangent.x == pytest.approx(Vector3(1.0, 0.0, 0.0).x)
    assert tangent.y == pytest.approx(Vector3(1.0, 0.0, 0.0).y)
    assert tangent.z == pytest.approx(Vector3(1.0, 0.0, 0.0).z)
    assert curvature == -1.0

    query_poistion = Vector3(1.0, 0.0, 0.0)
    (closest_point, tangent, curvature) = trajectory.get_closest_point(query_poistion)
    expected_closest_point = Vector3(
        math.sin(math.pi / 4.0), -1.0 + math.cos(math.pi / 4.0), 0.0
    )
    assert closest_point.x == pytest.approx(expected_closest_point.x)
    assert closest_point.y == pytest.approx(expected_closest_point.y)
    assert closest_point.z == pytest.approx(expected_closest_point.z)

    expected_closest_point = Vector3(
        math.sin(math.pi / 4.0), -1.0 + math.cos(math.pi / 4.0), 0.0
    )
    assert (closest_point - expected_closest_point).length() < FLOAT_EPS
    expected_tangent = Vector3(1.0, -1.0, 0.0).normalized()
    assert tangent.x == pytest.approx(expected_tangent.x)
    assert tangent.y == pytest.approx(expected_tangent.y)
    assert tangent.z == pytest.approx(expected_tangent.z)
    assert curvature == -1.0

    query_poistion = Vector3(2.0, -1.1, 0.0)
    (closest_point, tangent, curvature) = trajectory.get_closest_point(query_poistion)
    assert closest_point == Vector3(1.0, -1.1, 0.0)
    assert tangent == Vector3(0.0, -1.0, 0.0)
    assert curvature == 0.0
