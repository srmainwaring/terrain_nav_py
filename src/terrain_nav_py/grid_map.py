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


# Mock interface for GridMap
class GridMap:

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
        self._distance_surface = 0.0
        self._max_elevation = 120.0
        self._size = 10

    def __iter__(self) -> Iterator:
        return GridMap.Iterator(self)

    def size(self) -> int:
        return self._size

    def getPosition(self) -> tuple[float, float]:
        return self._position

    def getLength(self) -> tuple[float, float]:
        return self._length

    def isInside(self, position: tuple[float, float]) -> bool:
        return True

    def atPosition(self, layer: str, position: tuple[float, float]) -> float:
        if layer == "elevation":
            return self._elevation
        elif layer == "distance_surface":
            return self._distance_surface
        elif layer == "max_elevation":
            return self._max_elevation
        else:
            return float("nan")

    def at(self, layer: str, index: Index) -> float:
        if layer == "elevation":
            return self._elevation
        elif layer == "distance_surface":
            return self._distance_surface
        elif layer == "max_elevation":
            return self._max_elevation
        else:
            return float("nan")
