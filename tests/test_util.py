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


from terrain_nav_py.util import wrap_2pi
from terrain_nav_py.util import wrap_pi


def test_wrap_2pi():
    for n in [-10, -1, 0, 1, 10]:
        angle_deg = math.degrees(wrap_2pi(math.radians(n * 360.0 + 1.0)))
        assert angle_deg == pytest.approx(1.0)

        angle_deg = math.degrees(wrap_2pi(math.radians(n * 360.0 + 90.0)))
        assert angle_deg == pytest.approx(90.0)

        angle_deg = math.degrees(wrap_2pi(math.radians(n * 360.0 + 180.0)))
        assert angle_deg == pytest.approx(180.0)

        angle_deg = math.degrees(wrap_2pi(math.radians(n * 360.0 + 270.0)))
        assert angle_deg == pytest.approx(270.0)

    # modulo 360, angle > 0
    angle_deg = math.degrees(wrap_2pi(math.radians(360.0)))
    assert angle_deg == pytest.approx(360.0)

    angle_deg = math.degrees(wrap_2pi(math.radians(720.0)))
    assert angle_deg == pytest.approx(360.0)

    angle_deg = math.degrees(wrap_2pi(math.radians(3600.0)))
    assert angle_deg == pytest.approx(360.0)

    # modulo 360, angle < 0
    angle_deg = math.degrees(wrap_2pi(math.radians(-360.0)))
    assert angle_deg == pytest.approx(0.0)

    angle_deg = math.degrees(wrap_2pi(math.radians(-720.0)))
    assert angle_deg == pytest.approx(0.0)

    angle_deg = math.degrees(wrap_2pi(math.radians(-3600.0)))
    assert angle_deg == pytest.approx(0.0)

    # boundary 0 -> 0
    angle_rad = wrap_2pi(0.0)
    assert angle_rad == pytest.approx(0.0)

    # boundary 2 pi -> 2 pi
    angle_rad = wrap_2pi(2.0 * math.pi)
    assert angle_rad == pytest.approx(2.0 * math.pi)


def test_wrap_pi():
    for n in [-10, -1, 0, 1, 10]:
        angle_deg = math.degrees(wrap_pi(math.radians(n * 360 + 1.0)))
        assert angle_deg == pytest.approx(1.0)

        angle_deg = math.degrees(wrap_pi(math.radians(n * 360 + 90.0)))
        assert angle_deg == pytest.approx(90.0)

        angle_deg = math.degrees(wrap_pi(math.radians(n * 360 + 179.0)))
        assert angle_deg == pytest.approx(179.0)

        angle_deg = math.degrees(wrap_pi(math.radians(n * 360 + 181.0)))
        assert angle_deg == pytest.approx(-179.0)

        angle_deg = math.degrees(wrap_pi(math.radians(n * 360 + 270.0)))
        assert angle_deg == pytest.approx(-90.0)

        angle_deg = math.degrees(wrap_pi(math.radians(n * 360 + 360.0)))
        assert angle_deg == pytest.approx(0.0)

        angle_deg = math.degrees(wrap_pi(math.radians(n * 360 + 361.0)))
        assert angle_deg == pytest.approx(1.0)

    # boundary -pi -> -pi
    angle_rad = wrap_pi(-math.pi)
    assert angle_rad == pytest.approx(-math.pi)

    # boundary pi -> pi
    angle_rad = wrap_pi(math.pi)
    assert angle_rad == pytest.approx(math.pi)
