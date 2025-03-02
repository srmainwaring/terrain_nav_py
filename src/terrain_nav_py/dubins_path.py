# BSD 3-Clause License
#
# Copyright (c) 2025, Rhys Mainwaring
#
# Ported to Python from original C++ code in
# https://github.com/ethz-asl/terrain-navigation.git
#
# Copyright (c) 2016, Daniel Schneider, ASL; Florian Achermann, ASL
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
Dubins path

Data structures used by the DubinsAirPlane model.
"""


import math

from enum import IntEnum
from typing import Self


class DubinsPath:
    """
    Complete description of a (non-optimal) Dubins airplane path
    """

    class DubinsPathSegmentType(IntEnum):
        """
        The Dubins car/airplane path segment type.
        """

        DUBINS_LEFT = 0
        DUBINS_STRAIGHT = 1
        DUBINS_RIGHT = 2

        def __str__(self):
            if self.value == DubinsPath.DubinsPathSegmentType.DUBINS_LEFT.value:
                return "DUBINS_LEFT"
            elif self.value == DubinsPath.DubinsPathSegmentType.DUBINS_STRAIGHT.value:
                return "DUBINS_STRAIGHT"
            elif self.value == DubinsPath.DubinsPathSegmentType.DUBINS_RIGHT.value:
                return "DUBINS_RIGHT"
            else:
                return "INVALID"

    class Classification(IntEnum):
        """
        Classification of path according to "Classification of the Dubins Set", Shkel, Lumelsky, 2001
        """

        CLASS_A11 = 0
        CLASS_A12 = 1
        CLASS_A13 = 2
        CLASS_A14 = 3
        CLASS_A21 = 4
        CLASS_A22 = 5
        CLASS_A23 = 6
        CLASS_A24 = 7
        CLASS_A31 = 8
        CLASS_A32 = 9
        CLASS_A33 = 10
        CLASS_A34 = 11
        CLASS_A41 = 12
        CLASS_A42 = 13
        CLASS_A43 = 14
        CLASS_A44 = 15
        NOT_ASSIGNED = 16

        def __str__(self):
            if self.value == DubinsPath.Classification.CLASS_A11.value:
                return "CLASS_A11"
            elif self.value == DubinsPath.Classification.CLASS_A12.value:
                return "CLASS_A12"
            elif self.value == DubinsPath.Classification.CLASS_A13.value:
                return "CLASS_A13"
            elif self.value == DubinsPath.Classification.CLASS_A14.value:
                return "CLASS_A14"
            if self.value == DubinsPath.Classification.CLASS_A21.value:
                return "CLASS_A21"
            elif self.value == DubinsPath.Classification.CLASS_A22.value:
                return "CLASS_A22"
            elif self.value == DubinsPath.Classification.CLASS_A23.value:
                return "CLASS_A23"
            elif self.value == DubinsPath.Classification.CLASS_A24.value:
                return "CLASS_A24"
            if self.value == DubinsPath.Classification.CLASS_A31.value:
                return "CLASS_A31"
            elif self.value == DubinsPath.Classification.CLASS_A32.value:
                return "CLASS_A32"
            elif self.value == DubinsPath.Classification.CLASS_A33.value:
                return "CLASS_A33"
            elif self.value == DubinsPath.Classification.CLASS_A34.value:
                return "CLASS_A34"
            if self.value == DubinsPath.Classification.CLASS_A41.value:
                return "CLASS_A41"
            elif self.value == DubinsPath.Classification.CLASS_A42.value:
                return "CLASS_A42"
            elif self.value == DubinsPath.Classification.CLASS_A43.value:
                return "CLASS_A43"
            elif self.value == DubinsPath.Classification.CLASS_A44.value:
                return "CLASS_A44"
            else:
                return "INVALID"

    class Index(IntEnum):
        """
        Type of the dubins path.
        """

        TYPE_LSL = 0
        TYPE_RSR = 1
        TYPE_RSL = 2
        TYPE_LSR = 3
        TYPE_RLR = 4
        TYPE_LRL = 5

        def __str__(self):
            if self.value == DubinsPath.Index.TYPE_LSL.value:
                return "TYPE_LSL"
            elif self.value == DubinsPath.Index.TYPE_RSR.value:
                return "TYPE_RSR"
            elif self.value == DubinsPath.Index.TYPE_RSL.value:
                return "TYPE_RSL"
            elif self.value == DubinsPath.Index.TYPE_LSR.value:
                return "TYPE_LSR"
            elif self.value == DubinsPath.Index.TYPE_RLR.value:
                return "TYPE_RLR"
            elif self.value == DubinsPath.Index.TYPE_LRL.value:
                return "TYPE_LRL"
            else:
                return "INVALID"

    class AltitudeCase(IntEnum):
        """
        Altitude case of the path.
        """

        ALT_CASE_LOW = 0
        ALT_CASE_MEDIUM = 1
        ALT_CASE_HIGH = 2

        def __str__(self):
            if self.value == DubinsPath.AltitudeCase.ALT_CASE_LOW.value:
                return "ALT_CASE_LOW"
            elif self.value == DubinsPath.AltitudeCase.ALT_CASE_MEDIUM.value:
                return "ALT_CASE_MEDIUM"
            elif self.value == DubinsPath.AltitudeCase.ALT_CASE_HIGH.value:
                return "ALT_CASE_HIGH"
            else:
                return "INVALID"

    # Dubins car path types
    dubinsPathType = [
        [
            DubinsPathSegmentType.DUBINS_LEFT,
            DubinsPathSegmentType.DUBINS_STRAIGHT,
            DubinsPathSegmentType.DUBINS_LEFT,
        ],
        [
            DubinsPathSegmentType.DUBINS_RIGHT,
            DubinsPathSegmentType.DUBINS_STRAIGHT,
            DubinsPathSegmentType.DUBINS_RIGHT,
        ],
        [
            DubinsPathSegmentType.DUBINS_RIGHT,
            DubinsPathSegmentType.DUBINS_STRAIGHT,
            DubinsPathSegmentType.DUBINS_LEFT,
        ],
        [
            DubinsPathSegmentType.DUBINS_LEFT,
            DubinsPathSegmentType.DUBINS_STRAIGHT,
            DubinsPathSegmentType.DUBINS_RIGHT,
        ],
        [
            DubinsPathSegmentType.DUBINS_RIGHT,
            DubinsPathSegmentType.DUBINS_LEFT,
            DubinsPathSegmentType.DUBINS_RIGHT,
        ],
        [
            DubinsPathSegmentType.DUBINS_LEFT,
            DubinsPathSegmentType.DUBINS_RIGHT,
            DubinsPathSegmentType.DUBINS_LEFT,
        ],
    ]

    def __init__(
        self,
        type=Index.TYPE_LRL,
        t: float = 0.0,
        p: float = float("nan"),
        q: float = 0.0,
        gam: float = 0.0,
        ks: int = 0,
        ke: int = 0,
        r: float = 1.0,
    ):
        """
        Constructor

        :param t: length of first path segment of a 2D Dubins car path, defaults to 0.0
        :type t: float
        :param p: length of second path segment of a 2D Dubins car path, defaults to NaN
        :type p: float
        :param q: length of third path segment of a 2D Dubins car path, defaults to 0.0
        :type q: float
        """
        # DubinsPath.Index.TYPE_LRL

        # Path segment types
        self._type = DubinsPath.dubinsPathType[type]

        # On x-y plane projected path segment lengths, normalized by minimum
        # radius rho_ ( (.)*rho_ gives length of projection of path segments
        # in meters) length_[1,3,4]: length of 2D Dubins car path segments
        # length_[0,5]: length of start/ end helix for optimality in high
        # altitude case length_[2]: length of intermediate maneuver (at start)
        # for optimality in intermediate altitude case
        self._length = [0.0, t, 0.0, p, q, 0.0]

        # The 2D length of the curve based on the values in the length_ array.
        self._length_2d: float = t + p + q

        # Radius ratio (R_opt/rho_) for each segment.
        # For high altitude case, the radius may be bigger than rho_ in order to
        # guarantee optimal paths
        self._radiusRatio = [r, r, r, r, r, r]

        # The inverse value of the radius ratio.
        self._radiusRatioInverse = [
            1.0 / r,
            1.0 / r,
            1.0 / r,
            1.0 / r,
            1.0 / r,
            1.0 / r,
        ]

        # Current path angle (positive for flying upwards)
        self._gamma: float = gam

        # Computed to speed up the length3D computation.
        # 1.0 / cos(fabs(gamma))
        self._one_div_cos_abs_gamma: float = 1.0 / math.cos(math.fabs(self._gamma))

        # Number of circles to fly in end-helix
        self._k_end: int = ke

        # Number of circles to fly in start-helix
        self._k_start: int = ks

        # Classification of path according to "Classification of the Dubins Set", Shkel, Lumelsky, 2001
        # If a is equal to 0, the class of the path corresponds to a_11 of the mentioned paper:
        #  0: a_11
        #  1: a_12
        #  2: a_13
        #  3: a_14,
        #  4: a_21,   ...
        #  ...
        #  12: a_41,  ...
        #  ...
        #  15: a_44
        #  16, the class is not assigned
        #
        self._classification: DubinsPath.Classification = DubinsPath.Classification.NOT_ASSIGNED

        # Same information as in type_:
        #   0: LSL
        #   1: RSR
        #   2: RSL
        #   3: LSR
        #   4: RLR
        #   5: LRL
        #
        self._idx: DubinsPath.Index = type

        # Altitude case of the path:
        #     low (0)
        #     (medium (1), never appears for this state space)
        #     high (2)
        #
        self._lmh: DubinsPath.AltitudeCase = DubinsPath.AltitudeCase.ALT_CASE_LOW

        # Indicates if the path consists of three parts (CCC and CSC) with a
        # value of 0 or of four parts CCSC with a value of 1
        self._additionalManeuver: bool = False

        # True if an optimal path was found, false if no or a suboptimal path was found.
        self._foundOptimalPath: bool = True

    def length_2d(self) -> float:
        """
        A function returning the length (normalized by minimum radius rho_) of the projection of the
        3D (non-optimal) Dubins airplane path on the x-y plane of the world frame.
        """
        return self._length_2d

    def length_3d(self) -> float:
        """
        A function returning the length (normalized by minimum radius) of the 3D (non-optimal) Dubins airplane
        path.
        """
        return self.length_2d() * self._one_div_cos_abs_gamma

    def getFoundOptimalPath(self) -> bool:
        """
        Return foundOptimalPath_
        """
        return self._foundOptimalPath

    def setFoundOptimalPath(self, found_optimal_path: bool) -> None:
        """
        Set foundOptimalPath_
        """
        self._foundOptimalPath = found_optimal_path

    def getAdditionalManeuver(self) -> bool:
        """
        Return additionalManeuver_
        """
        return self._additionalManeuver

    def setAdditionalManeuver(self, additional_maneuver: bool) -> None:
        """
        Set additionalManeuver_
        """
        self._additionalManeuver = additional_maneuver

    def getAltitudeCase(self) -> AltitudeCase:
        """
        Return lmh_
        """
        return self._lmh

    def setAltitudeCase(self, altitude_case: AltitudeCase) -> None:
        """
        Set lmh_
        """
        self._lmh = altitude_case

    def getIdx(self) -> Index:
        """
        Return idx_
        """
        return self._idx

    def setClassification(self, classification: Classification) -> None:
        """
        Set classification_
        """
        self._classification = classification

    def getClassification(self) -> Classification:
        """
        Return classification_
        """
        return self._classification

    def setStartHelix(self, num_helix: int, radius_ratio: float) -> None:
        """
        Set a start helix with num_helix full circles and a radius ratio of radius_ratio.
        """
        self._k_start = num_helix
        self._radiusRatio[0] = radius_ratio
        self._radiusRatioInverse[0] = 1.0 / radius_ratio
        self._length[0] = num_helix * 2.0 * math.pi * radius_ratio
        self._length_2D = (
            self._length[0]
            + self._length[1]
            + self._length[2]
            + self._length[3]
            + self._length[4]
            + self._length[5]
        )

    def setEndHelix(self, num_helix: int, radius_ratio: float) -> None:
        """
        Set a end helix with num_helix full circles and a radius ratio of radius_ratio.
        """
        self._k_end = num_helix
        self._radiusRatio[5] = radius_ratio
        self._radiusRatioInverse[5] = 1.0 / radius_ratio
        self._length[5] = num_helix * 2.0 * math.pi * radius_ratio
        self._length_2D = (
            self._length[0]
            + self._length[1]
            + self._length[2]
            + self._length[3]
            + self._length[4]
            + self._length[5]
        )

    def setGamma(self, gamma: float) -> None:
        """
        Set gamma_
        """
        self._gamma = gamma
        self._one_div_cos_abs_gamma = 1.0 / math.cos(math.fabs(self._gamma))

    def getGamma(self) -> float:
        """
        Return gamma_
        """
        return self._gamma

    def getRadiusRatio(self, idx: int) -> float:
        """
        Return radiusRatio_ of the corresponding index, the index must be between 0 and 5.
        """
        return self._radiusRatio[idx]

    def getInverseRadiusRatio(self, idx: int) -> float:
        """
        Return radiusRatioInverse_ of the corresponding index, the index must be between 0 and 5.
        """
        return self._radiusRatioInverse[idx]

    def getSegmentLength(self, idx: int) -> float:
        """
        Return length_ of the corresponding index, the index must be between 0 and 5.
        """
        return self._length[idx]

    def setSegmentLength(self, length: float, idx: int) -> None:
        """
        Set length_ of the corresponding index, the index must be between 0 and 5.
        """
        self._length[idx] = length
        self._length_2D = (
            self._length[0]
            + self._length[1]
            + self._length[2]
            + self._length[3]
            + self._length[4]
            + self._length[5]
        )

    def getType(self) -> DubinsPathSegmentType:
        """
        Return type_
        """
        return self._type
