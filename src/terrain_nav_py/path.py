# BSD 3-Clause License
#
# Copyright (c) 2025, Rhys Mainwaring
#
# Ported to Python from original C++ code in
# https://github.com/ethz-asl/terrain-navigation.git
#
# Copyright (c) 2021-2023 Jaeyoung Lim, Autonomous Systems Lab,
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
Generate paths for guided path navigation
"""

from terrain_nav_py.path_segment import PathSegment

from typing import Self

# Use spatial objects defined in pymavlink
from pymavlink.rotmat import Vector3


class Path:
    """
    A path comprising a vector of path segments.
    """

    def __init__(self):
        self._segments = []
        self._utility = 0.0
        self._is_valid = False
        self._epsilon = 15.0 * 0.2

    @property
    def is_valid(self) -> bool:
        return self._is_valid

    def position(self) -> list[Vector3]:
        result = []
        for seg in self._segments:
            pos = seg.position()
            result.extend(pos)
        return result

    def velocity(self) -> list[Vector3]:
        result = []
        for seg in self._segments:
            vel = seg.velocity()
            result.extend(vel)
        return result

    def attitude(self) -> list[Vector3]:
        result = []
        for seg in self._segments:
            att = seg.attitude()
            result.extend(att)
        return result

    def reset_segments(self) -> None:
        self._segments.clear()

    def prepend_segment(self, path_segment: PathSegment) -> None:
        self._segments.insert(0, path_segment)

    def append_segment(self, path_segment: PathSegment) -> None:
        self._segments.append(path_segment)

    def append_path(self, path: Self) -> None:
        for seg in path._segments:
            self.append_segment(seg)

    def first_segment(self) -> PathSegment:
        return self._segments[0]

    def last_segment(self) -> PathSegment:
        return self._segments[-1]

    def get_closest_point(self, position: Vector3) -> tuple[Vector3, Vector3, float]:

        closest_point = Vector3()
        tangent = Vector3()
        curvature = 0.0

        closest_point = self.first_segment().first_state().position

        # iterate through all segments
        for segment in self._segments:
            if segment.reached and (segment is not self._segments[-1]):
                continue
            (theta, closest_point, tangent, curvature) = segment.get_closest_point(
                position
            )

            curvature = segment._curvature
            if theta <= 0.0:
                # theta can be negative on the start of the next segment
                closest_point = segment.first_state().position
                tangent = (segment.first_state().velocity).normalized()
                return (closest_point, tangent, curvature)
            elif theta < 1.0:
                # TODO: This is a magic number in terms of acceptance radius of end of segments
                return (closest_point, tangent, curvature)
            else:
                closest_point = segment.last_state().position
                tangent = (segment.last_state().velocity).normalized()
                # consider that the segment is tracked
                segment.reached = True

        # TODO: Handle the case when all segments are marked as reached
        return (closest_point, tangent, curvature)

    def get_end_of_current_segment(self, position: Vector3) -> Vector3:
        if self._segments.empty():
            return position
        else:
            pos = self.get_current_segment(position).last_state().position
            return pos

    def get_current_segment(self, position: Vector3) -> PathSegment:
        closest_point = Vector3
        tangent = Vector3
        curvature = 0.0

        segment_idx = -1
        for segment in self._segments:
            segment_idx += 1
            if segment.reached and (segment is not self._segments.back()):
                continue

            theta = segment.get_closest_point(
                position, closest_point, tangent, curvature
            )

            # If current segment is a full circle, and has a next segment,
            # escape when close to start of next segment
            if segment.is_periodic and (segment is not self._segments.back()):
                # Segment is a terminal periodic set
                next_segment_start = self._segments[segment_idx].states.front().position
                if (closest_point - next_segment_start).length() < self._epsilon:
                    segment.reached = True
                    return segment

            if theta <= 0.0:
                return segment
            elif theta < 1.0:
                return segment
            else:
                segment.reached = True

    def get_current_segment_index(self, position: Vector3) -> int:
        closest_point = Vector3
        tangent = Vector3
        curvature = 0.0

        #  Iterate through all segments
        segment_idx = -1
        for segment in self._segments:
            segment_idx += 1
            if segment.reached and (segment is not self._segments.back()):
                continue
            theta = segment.get_closest_point(
                position, closest_point, tangent, curvature
            )
            if theta <= 0.0:
                # theta can be negative on the start of the next segment
                return segment_idx
            elif theta < 1.0:
                # TODO: This is a magic number in terms of acceptance
                #       radius of end of segments
                return segment_idx

        return segment_idx

    def get_length(self, start_idx: int = 0) -> float:
        length = 0.0
        for i in range(start_idx, len(self._segments)):
            length += self._segments[i].get_length()

        return length
