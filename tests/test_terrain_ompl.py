import math
import pytest
import sys

from ompl import base as ob
from ompl import geometric as og

from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace

from terrain_nav_py.terrain_ompl import GridMap
from terrain_nav_py.terrain_ompl import TerrainValidityChecker
from terrain_nav_py.terrain_ompl import TerrainStateSampler


def test_terrain_ompl_gridmap_position():
    map = GridMap()

    pos = map.getPosition()
    assert pos[0] == 0.0
    assert pos[1] == 0.0


def test_terrain_ompl_gridmap_length():
    map = GridMap()

    len = map.getLength()
    assert len[0] == 1000.0
    assert len[1] == 1000.0


def test_terrain_ompl_gridmap_is_inside():
    map = GridMap()

    pos = (100.0, 100.0)
    assert map.isInside(pos) == True


def test_terrain_ompl_gridmap_at_position():
    map = GridMap()

    pos = (100.0, 100.0)
    assert map.atPosition("elevation", pos) == 0.0
    assert map.atPosition("max_elevation", pos) == 120.0
    assert map.atPosition("distance_surface", pos) == 0.0


def test_terrain_ompl_gridmap_iterator():
    map = GridMap()

    for i, index in enumerate(map):
        assert index.value == i
        # print(f"index: {index.value}")


def test_terrain_ompl_gridmap_at():
    map = GridMap()

    for index in map:
        assert map.atPosition("elevation", index) == 0.0
        assert map.atPosition("max_elevation", index) == 120.0
        assert map.atPosition("distance_surface", index) == 0.0


def test_terrain_ompl_validity_checker_collision():
    # gridmap
    map = GridMap()

    # state space
    space = ob.CompoundStateSpace()
    space.addSubspace(ob.RealVectorStateSpace(3), 1.0)
    space.addSubspace(ob.SO2StateSpace(), 1.0)
    space.lock()

    ss = og.SimpleSetup(space)
    si = ss.getSpaceInformation()

    checker = TerrainValidityChecker(si, map, check_max_altitude=True)

    # collision
    pos = (100.0, 200.0, 60.0)

    # check above/below elevation
    hit = checker.isInCollision("elevation", pos, is_above=True)
    assert hit == False

    hit = checker.isInCollision("elevation", pos, is_above=False)
    assert hit == True

    # check above/below distance surface
    hit = checker.isInCollision("distance_surface", pos, is_above=True)
    assert hit == False

    hit = checker.isInCollision("distance_surface", pos, is_above=False)
    assert hit == True

    # check above/below max elevation
    hit = checker.isInCollision("max_elevation", pos, is_above=True)
    assert hit == True

    hit = checker.isInCollision("max_elevation", pos, is_above=False)
    assert hit == False


def test_terrain_ompl_validity_checker_collision():
    # gridmap
    map = GridMap()

    # state space
    space = DubinsAirplaneStateSpace()

    ss = og.SimpleSetup(space)
    si = ss.getSpaceInformation()

    checker = TerrainValidityChecker(si, map, check_max_altitude=True)

    # initialise a state and use DubinsAirplaneStateSpace.StateWrapper
    state = ob.State(space)
    da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)

    da_state.setX(100.0)
    da_state.setY(200.0)
    da_state.setZ(60.0)
    da_state.setYaw(0.0)
    assert checker.isValid(state) == True

    da_state.setX(100.0)
    da_state.setY(200.0)
    da_state.setZ(-60.0)
    da_state.setYaw(0.0)
    assert checker.isValid(state) == False


def test_terrain_ompl_sampler_uniform():
    # gridmap
    map = GridMap()

    # state space
    space = DubinsAirplaneStateSpace()
    sampler = TerrainStateSampler(space, map, max_altitude=120.0, min_altitude=60.0)
    state = ob.State(space)
    da_state = DubinsAirplaneStateSpace.DubinsAirplaneState(state)

    # NOTE: may get sample of zero - but not likely.
    sampler.sampleUniform(state)
    assert da_state.getX() != 0.0
    assert da_state.getY() != 0.0
    assert da_state.getZ() != 0.0
    assert da_state.getYaw() != 0.0


if __name__ == "__main__":
    test_terrain_ompl_gridmap_iterator()
