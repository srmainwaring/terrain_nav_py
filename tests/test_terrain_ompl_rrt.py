import math
import pytest
import sys

from ompl import base as ob
from ompl import geometric as og

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace

from terrain_nav_py.terrain_ompl import GridMap

# from terrain_nav_py.terrain_ompl_rrt import TerrainValidityChecker


def test_terrain_ompl_rrt():
    map = GridMap()

    pos = map.getPosition()
    assert pos[0] == 0.0
    assert pos[1] == 0.0
