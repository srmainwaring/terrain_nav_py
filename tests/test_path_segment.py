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
from terrain_nav_py.util import wrap_2pi

FLOAT_EPS = 1.0e-6


def test_path_segment_get_arc_centre1():
    segment_start = Vector3(10.0, 10.0, 0.0)
    segment_start_tangent = Vector3(1.0, -1.0, 0.0) / math.sqrt(2.0)
    radius = math.sqrt(2.0) * 10.0
    curvature = 1.0 / radius

    assert segment_start.x == 10.0
    assert segment_start.y == 10.0
    assert segment_start_tangent.x == 1.0 / math.sqrt(2.0)
    assert segment_start_tangent.y == -1.0 / math.sqrt(2.0)

    arc_centre = PathSegment.get_arc_centre(
        segment_start, segment_start_tangent, curvature
    )
    assert arc_centre.x == 20.0
    assert arc_centre.y == 20.0

    arc_centre = PathSegment.get_arc_centre(
        segment_start, segment_start_tangent, -1.0 * curvature
    )
    assert arc_centre.x == 0.0
    assert arc_centre.y == 0.0


def test_path_segment_get_arc_centre2():
    segment_start = Vector3(10.0, 10.0, 0.0)
    segment_end = Vector3(20.0, 20.0 - 10.0 * math.sqrt(2.0), 0.0)
    segment_start_tangent = Vector3(1.0, -1.0, 0.0) / math.sqrt(2.0)
    radius = math.sqrt(2.0) * 10.0
    curvature = 1.0 / radius

    assert segment_start.x == 10.0
    assert segment_start.y == 10.0
    assert segment_end.x == 20.0
    assert segment_end.y == 20.0 - 10.0 * math.sqrt(2.0)
    assert segment_start_tangent.x == 1.0 / math.sqrt(2.0)
    assert segment_start_tangent.y == -1.0 / math.sqrt(2.0)

    arc_centre = PathSegment.get_arc_centre2(
        segment_start, segment_start_tangent, curvature, segment_end
    )
    assert arc_centre.x == 20.0
    assert arc_centre.y == 20.0


def test_path_segment_get_line_progress1():
    segment_start = Vector3(10.0, 10.0, 0.0)
    segment_end = Vector3(20.0, 20.0, 0.0)

    position = Vector3(11.0, 11.0, 0.0)
    theta = PathSegment.get_line_progress(position, segment_start, segment_end)
    assert theta == pytest.approx(0.1)

    position = Vector3(15.0, 15.0, 0.0)
    theta = PathSegment.get_line_progress(position, segment_start, segment_end)
    assert theta == pytest.approx(0.5)

    position = Vector3(18.0, 18.0, 0.0)
    theta = PathSegment.get_line_progress(position, segment_start, segment_end)
    assert theta == pytest.approx(0.8)


def test_path_segment_get_arc_progress0():
    arc_centre = Vector3(20.0, 20.0, 0.0)
    segment_start = Vector3(10.0, 10.0, 0.0)
    segment_end = Vector3(30.0, 10.0, 0.0)
    radius = 10.0 * math.sqrt(2.0)
    curvature = 1.0 / radius

    position = Vector3(10.0, 10.0, 0.0)
    assert position == segment_start
    theta = PathSegment.get_arc_progress(
        arc_centre, position, segment_start, segment_end, curvature
    )
    assert theta == pytest.approx(0.0)

    position = Vector3(30.0, 10.0, 0.0)
    assert position == segment_end
    theta = PathSegment.get_arc_progress(
        arc_centre, position, segment_start, segment_end, curvature
    )
    assert theta == pytest.approx(1.0)

    position = Vector3(20.0, 20.0 - radius, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_centre, position, segment_start, segment_end, curvature
    )
    assert theta == pytest.approx(0.5)


def test_path_segment_get_length():

    # 1. line - along x-axis
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, 0.0)

    state1 = State()
    state1.position = Vector3(10.0, 10.0, 10.0)
    state1.velocity = Vector3(5.0, 0.0, 0.0)
    state1.attitude = quaternion.Quaternion(m_att)

    state2 = State()
    state2.position = Vector3(30.0, 10.0, 10.0)
    state2.velocity = Vector3(5.0, 0.0, 0.0)
    state2.attitude = quaternion.Quaternion(m_att)

    path_segment = PathSegment()
    path_segment.curvature = 0.0
    path_segment.is_periodic = False
    path_segment.reached = False
    path_segment.append_state(state1)
    path_segment.append_state(state2)

    pos_vec = path_segment.position()
    vel_vec = path_segment.velocity()
    att_vec = path_segment.attitude()

    # check state vector lengths
    assert len(pos_vec) == 2
    assert len(vel_vec) == 2
    assert len(att_vec) == 2

    # check state vector contents
    assert pos_vec[0] == state1.position
    assert vel_vec[0] == state1.velocity
    assert att_vec[0] == state1.attitude
    assert pos_vec[1] == state2.position
    assert vel_vec[1] == state2.velocity
    assert att_vec[1] == state2.attitude

    # check first state
    assert path_segment.first_state().position == state1.position
    assert path_segment.first_state().velocity == state1.velocity
    assert path_segment.first_state().attitude == state1.attitude

    # check last state
    assert path_segment.last_state().position == state2.position
    assert path_segment.last_state().velocity == state2.velocity
    assert path_segment.last_state().attitude == state2.attitude

    length = path_segment.get_length()
    assert length == 20.0

    # 2. partial arc - around centre at (20, 20, 10)
    radius = 10.0 * math.sqrt(2.0)
    curvature = 1 / radius

    # heading: 135
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, math.radians(135.0))

    state1 = State()
    state1.position = Vector3(10.0, 10.0, 10.0)
    state1.velocity = Vector3(5.0, -5.0, 0.0)
    state1.attitude = quaternion.Quaternion(m_att)

    # heading: 45
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, math.radians(45.0))

    state2 = State()
    state2.position = Vector3(30.0, 10.0, 10.0)
    state2.velocity = Vector3(5.0, 5.0, 0.0)
    state2.attitude = quaternion.Quaternion(m_att)

    path_segment = PathSegment()
    path_segment.curvature = curvature
    path_segment.is_periodic = False
    path_segment.reached = False
    path_segment.append_state(state1)
    path_segment.append_state(state2)

    # expected arc length is 1/4 circumference
    length = path_segment.get_length()
    assert length == 0.25 * 2.0 * math.pi * radius

    # 2. full circle arc with radius 100.0
    radius = 100.0
    curvature = 1 / radius

    # heading: 135
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, math.radians(135.0))

    state1 = State()
    state1.position = Vector3(10.0, 10.0, 10.0)
    state1.velocity = Vector3(5.0, -5.0, 0.0)
    state1.attitude = quaternion.Quaternion(m_att)

    # heading: 135
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, math.radians(135.0))

    state2 = State()
    state2.position = Vector3(10.0, 10.0, 10.0)
    state2.velocity = Vector3(5.0, -5.0, 0.0)
    state2.attitude = quaternion.Quaternion(m_att)

    path_segment = PathSegment()
    path_segment.curvature = curvature
    path_segment.is_periodic = True
    path_segment.reached = False
    path_segment.append_state(state1)
    path_segment.append_state(state2)

    # expected arc length is circumference
    length = path_segment.get_length()
    assert length == 2.0 * math.pi * radius


def test_path_segment_get_closest_point():
    # 1. line - along x-axis
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, 0.0)

    state1 = State()
    state1.position = Vector3(10.0, 10.0, 10.0)
    state1.velocity = Vector3(5.0, 0.0, 0.0)
    state1.attitude = quaternion.Quaternion(m_att)

    state2 = State()
    state2.position = Vector3(30.0, 10.0, 10.0)
    state2.velocity = Vector3(5.0, 0.0, 0.0)
    state2.attitude = quaternion.Quaternion(m_att)

    path_segment = PathSegment()
    path_segment.curvature = 0.0
    path_segment.is_periodic = False
    path_segment.reached = False
    path_segment.append_state(state1)
    path_segment.append_state(state2)

    # closest point is mid-point
    position = Vector3(20.0, 15.0, 5.0)
    (theta, closest_point, tangent, curvature) = path_segment.get_closest_point(
        position
    )
    assert theta == 0.5
    assert closest_point == Vector3(20.0, 10.0, 10.0)
    assert tangent == Vector3(1.0, 0.0, 0.0)
    assert curvature == 0.0

    # closest point is segment start (point is before start)
    # TODO: confirm theta is not expected to be constrained - in this case
    #       no progress has been made along the line
    #       (so should be zero rather than negative?)
    position = Vector3(5.0, 15.0, 5.0)
    (theta, closest_point, tangent, curvature) = path_segment.get_closest_point(
        position
    )
    assert theta == -0.25
    assert closest_point == Vector3(10.0, 10.0, 10.0)
    assert tangent == Vector3(1.0, 0.0, 0.0)
    assert curvature == 0.0

    # closest point is segment end (point is after end)
    # TODO: confirm theta is not expected to be constrained - see above
    position = Vector3(35.0, 15.0, 5.0)
    (theta, closest_point, tangent, curvature_out) = path_segment.get_closest_point(
        position
    )
    assert theta == 1.25
    assert closest_point == Vector3(30.0, 10.0, 10.0)
    assert tangent == Vector3(1.0, 0.0, 0.0)
    assert curvature_out == 0.0

    # 2. partial arc - around centre at (20, 20, 10)
    radius = 10.0 * math.sqrt(2.0)
    curvature = 1 / radius

    # heading: 135
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, math.radians(135.0))

    state1 = State()
    state1.position = Vector3(10.0, 10.0, 10.0)
    state1.velocity = Vector3(5.0, -5.0, 0.0)
    state1.attitude = quaternion.Quaternion(m_att)

    # heading: 45
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, math.radians(45.0))

    state2 = State()
    state2.position = Vector3(30.0, 10.0, 10.0)
    state2.velocity = Vector3(5.0, 5.0, 0.0)
    state2.attitude = quaternion.Quaternion(m_att)

    path_segment = PathSegment()
    path_segment.curvature = curvature
    path_segment.is_periodic = False
    path_segment.reached = False
    path_segment.append_state(state1)
    path_segment.append_state(state2)

    # closest point is mid-point
    position = Vector3(20.0, 0.0, 5.0)
    (theta, closest_point, tangent, curvature_out) = path_segment.get_closest_point(
        position
    )
    assert theta == 0.5
    assert closest_point == Vector3(20.0, 20.0 - 10.0 * math.sqrt(2.0), 10.0)
    assert tangent == Vector3(1.0, 0.0, 0.0).normalized()
    assert curvature_out == curvature

    # closest point is segment start (point is before start)
    position = Vector3(0.0, 0.0, 5.0)
    (theta, closest_point, tangent, curvature_out) = path_segment.get_closest_point(
        position
    )
    assert theta == 0.0
    assert closest_point == Vector3(10.0, 10.0, 10.0)
    assert tangent == Vector3(1.0, -1.0, 0.0).normalized()
    assert curvature_out == curvature

    # closest point is segment end (point is after end)
    position = Vector3(40.0, 0.0, 5.0)
    (theta, closest_point, tangent, curvature_out) = path_segment.get_closest_point(
        position
    )
    assert theta == 1.0
    assert closest_point == Vector3(30.0, 10.0, 10.0)
    assert tangent == Vector3(1.0, 1.0, 0.0).normalized()
    assert curvature_out == curvature

    # 2. full circle arc with radius 100.0
    radius = 100.0
    curvature = 1 / radius

    # heading: 135
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, math.radians(135.0))

    state1 = State()
    state1.position = Vector3(10.0, 10.0, 10.0)
    state1.velocity = Vector3(5.0, -5.0, 0.0)
    state1.attitude = quaternion.Quaternion(m_att)

    # heading: 135
    m_att = Matrix3()
    m_att.from_euler(0.0, 0.0, math.radians(135.0))

    state2 = State()
    state2.position = Vector3(10.0, 10.0, 10.0)
    state2.velocity = Vector3(5.0, -5.0, 0.0)
    state2.attitude = quaternion.Quaternion(m_att)

    path_segment = PathSegment()
    path_segment.curvature = curvature
    path_segment.is_periodic = True
    path_segment.reached = False
    path_segment.append_state(state1)
    path_segment.append_state(state2)

    # closest point is mid-point - progess is 1/8 circumference
    arc_centre = state1.position + radius * Vector3(1.0, 1.0, 0.0).normalized()
    position = Vector3(arc_centre.x, 0.0, 5.0)
    (theta, closest_point, tangent, curvature_out) = path_segment.get_closest_point(
        position
    )
    assert theta == pytest.approx(0.125)
    assert closest_point.x == pytest.approx(
        Vector3(arc_centre.x, arc_centre.y - radius, 10.0).x
    )
    assert closest_point.y == pytest.approx(
        Vector3(arc_centre.x, arc_centre.y - radius, 10.0).y
    )
    assert closest_point.z == pytest.approx(
        Vector3(arc_centre.x, arc_centre.y - radius, 10.0).z
    )
    assert tangent.x == pytest.approx(Vector3(1.0, 0.0, 0.0).normalized().x)
    assert tangent.y == pytest.approx(Vector3(1.0, 0.0, 0.0).normalized().y)
    assert tangent.z == pytest.approx(Vector3(1.0, 0.0, 0.0).normalized().z)
    assert curvature_out == curvature


def test_path_segment_get_arc_center3():
    segment_start = Vector3(0.0, 0.0, 0.0)

    segment_start_tangent = Vector3(1.0, 0.0, 0.0)
    arc_center = PathSegment.get_arc_centre(segment_start, segment_start_tangent, 1.0)
    assert arc_center == Vector3(0.0, 1.0, 0.0)

    segment_start_tangent = Vector3(-1.0, 0.0, 0.0)
    arc_center2 = PathSegment.get_arc_centre(segment_start, segment_start_tangent, -1.0)
    assert arc_center2 == Vector3(0.0, 1.0, 0.0)


def test_path_segment_get_arc_progress1():
    segment_start = Vector3(0.0, 0.0, 0.0)
    segment_start_tangent = Vector3(1.0, 0.0, 0.0)
    segment_end = Vector3(0.0, 2.0, 0.0)
    expected_center = Vector3(0.0, 1.0, 0.0)

    arc_center = PathSegment.get_arc_centre(segment_start, segment_start_tangent, 1.0)
    theta = PathSegment.get_arc_progress(
        arc_center, segment_start, segment_start, segment_end, 1.0
    )
    assert theta == 0.0
    assert arc_center == expected_center
    theta = PathSegment.get_arc_progress(
        arc_center, segment_end, segment_start, segment_end, 1.0
    )
    assert theta == 1.0
    theta = PathSegment.get_arc_progress(
        arc_center, segment_start, segment_start, segment_end, -1.0
    )
    assert theta == 0.0
    theta = PathSegment.get_arc_progress(
        arc_center, segment_end, segment_start, segment_end, -1.0
    )
    assert theta == 1.0

    test_position = Vector3(0.0, -1.0, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, 1.0
    )
    assert arc_center == expected_center
    assert theta == 0.0
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, -1.0
    )
    assert theta == 0.0


def test_path_segment_get_arc_progress2():
    # Test close to full circle arc segments
    segment_start = Vector3(1.0, 0.0, 0.0)
    segment_start_tangent = Vector3(0.0, 1.0, 0.0)
    end_theta = 1.75 * math.pi
    segment_end = Vector3(math.cos(end_theta), math.sin(end_theta), 0.0)
    expected_center = Vector3(0.0, 0.0, 0.0)

    segment_end2 = Vector3(1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0)

    print(f"segment_start:          {segment_start}")
    print(f"segment_start_tangent:  {segment_start_tangent}")
    print(f"end_theta:              {end_theta}")
    print(f"segment_end:            {segment_end}")
    print(f"segment_end2:           {segment_end2}")
    print(f"expected_center:        {expected_center}")

    assert (segment_end - segment_end2).length() == pytest.approx(0.0)

    arc_center = PathSegment.get_arc_centre(segment_start, segment_start_tangent, 1.0)
    assert (arc_center - expected_center).length() < FLOAT_EPS

    theta = PathSegment.get_arc_progress(
        arc_center, segment_start, segment_start, segment_end, 1.0
    )
    assert theta == 0.0

    theta = PathSegment.get_arc_progress(
        arc_center, segment_end, segment_start, segment_end, 1.0
    )
    assert theta == 1.0

    test_position = Vector3(2.0, 0.0, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, 1.0
    )
    assert theta == 0.0

    test_position = Vector3(0.0, 2.0, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, 1.0
    )
    assert theta == pytest.approx(2.0 / 7.0)


def test_path_segment_get_arc_progress3():
    # Test close to full circle arc segments
    segment_start = Vector3(1.0, 0.0, 0.0)
    segment_start_tangent = Vector3(0.0, -1.0, 0.0)
    end_theta = -1.75 * math.pi
    segment_end = Vector3(math.cos(end_theta), math.sin(end_theta), 0.0)
    expected_center = Vector3(0.0, 0.0, 0.0)

    arc_center = PathSegment.get_arc_centre(segment_start, segment_start_tangent, -1.0)
    assert (arc_center - expected_center).length() < FLOAT_EPS

    theta = PathSegment.get_arc_progress(
        arc_center, segment_start, segment_start, segment_end, -1.0
    )
    assert theta == 0.0

    theta = PathSegment.get_arc_progress(
        arc_center, segment_end, segment_start, segment_end, -1.0
    )
    assert theta == 1.0

    test_position = Vector3(2.0, 0.0, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, -1.0
    )
    assert theta == 0.0

    test_position = Vector3(0.0, 2.0, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, -1.0
    )
    assert theta == pytest.approx(6.0 / 7.0)

    test_position = Vector3(1.0, 1.0, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, -1.0
    )
    assert theta == pytest.approx(1.0)


def test_path_segment_get_arc_progress4():
    # Test close to full circle arc segments
    segment_start = Vector3(1.0, 0.0, 0.0)
    segment_start_tangent = Vector3(0.0, -1.0, 0.0)
    end_theta = -1.5 * math.pi
    segment_end = Vector3(math.cos(end_theta), math.sin(end_theta), 0.0)
    expected_center = Vector3(0.0, 0.0, 0.0)

    arc_center = PathSegment.get_arc_centre(segment_start, segment_start_tangent, -1.0)
    assert (arc_center - expected_center).length() < FLOAT_EPS

    theta = PathSegment.get_arc_progress(
        arc_center, segment_start, segment_start, segment_end, -1.0
    )
    assert theta == pytest.approx(0.0)

    theta = PathSegment.get_arc_progress(
        arc_center, segment_end, segment_start, segment_end, -1.0
    )
    assert theta == pytest.approx(1.0)

    test_position = Vector3(2.0, 0.0, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, -1.0
    )
    assert theta == pytest.approx(0.0)

    test_position = Vector3(0.0, 2.0, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, -1.0
    )
    assert theta == pytest.approx(1.0)

    test_position = Vector3(1.0, 1.0, 0.0)
    theta = PathSegment.get_arc_progress(
        arc_center, test_position, segment_start, segment_end, -1.0
    )
    assert theta == pytest.approx(1.1666666666666667)


def test_path_segment_get_line_progress2():
    segment_start = Vector3(0.0, 0.0, 0.0)
    segment_end = Vector3(0.0, 0.0, 1.0)
    progress = Vector3(0.0, 0.0, 0.5)

    theta = PathSegment.get_line_progress(progress, segment_start, segment_end)
    assert theta == 0.5

    theta = PathSegment.get_line_progress(segment_start, segment_start, segment_end)
    assert theta == 0.0

    theta = PathSegment.get_line_progress(segment_end, segment_start, segment_end)
    assert theta == 1.0


def test_path_segment_get_length_angle_calc():
    t = Vector3(0, 1, 0)
    a = Vector3(1, 0, 0)
    b = Vector3(0, 1, 0)
    dir = (a % t).normalized().z
    psi_a = math.atan2(a.y, a.x)
    psi_b = math.atan2(b.y, b.x)
    psi_ab = dir * (psi_b - psi_a)
    psi_ab = wrap_2pi(psi_ab)
    print(f"dir: {dir:.1f}, psi: {math.degrees(psi_ab):.1f}")

    t = Vector3(0, 1, 0)
    a = Vector3(1, 0, 0)
    b = Vector3(-1, 0, 0)
    dir = (a % t).normalized().z
    psi_a = math.atan2(a.y, a.x)
    psi_b = math.atan2(b.y, b.x)
    psi_ab = dir * (psi_b - psi_a)
    psi_ab = wrap_2pi(psi_ab)
    print(f"dir: {dir:.1f}, psi: {math.degrees(psi_ab):.1f}")

    t = Vector3(0, 1, 0)
    a = Vector3(1, 0, 0)
    b = Vector3(0, -1, 0)
    dir = (a % t).normalized().z
    psi_a = math.atan2(a.y, a.x)
    psi_b = math.atan2(b.y, b.x)
    psi_ab = dir * (psi_b - psi_a)
    psi_ab = wrap_2pi(psi_ab)
    print(f"dir: {dir:.1f}, psi: {math.degrees(psi_ab):.1f}")

    t = Vector3(0, 1, 0)
    a = Vector3(-1, 0, 0)
    b = Vector3(0, 1, 0)
    dir = (a % t).normalized().z
    psi_a = math.atan2(a.y, a.x)
    psi_b = math.atan2(b.y, b.x)
    psi_ab = dir * (psi_b - psi_a)
    psi_ab = wrap_2pi(psi_ab)
    print(f"dir: {dir:.1f}, psi: {math.degrees(psi_ab):.1f}")

    t = Vector3(0, 1, 0)
    a = Vector3(-1, 0, 0)
    b = Vector3(0, -1, 0)
    dir = (a % t).normalized().z
    psi_a = math.atan2(a.y, a.x)
    psi_b = math.atan2(b.y, b.x)
    psi_ab = dir * (psi_b - psi_a)
    psi_ab = wrap_2pi(psi_ab)
    print(f"dir: {dir:.1f}, psi: {math.degrees(psi_ab):.1f}")

    t = Vector3(0, -1, 0)
    a = Vector3(-1, 0, 0)
    b = Vector3(0, 1, 0)
    dir = (a % t).normalized().z
    psi_a = math.atan2(a.y, a.x)
    psi_b = math.atan2(b.y, b.x)
    psi_ab = dir * (psi_b - psi_a)
    psi_ab = wrap_2pi(psi_ab)
    print(f"dir: {dir:.1f}, psi: {math.degrees(psi_ab):.1f}")


def main():
    test_path_segment_get_length_angle_calc()


if __name__ == "__main__":
    main()
