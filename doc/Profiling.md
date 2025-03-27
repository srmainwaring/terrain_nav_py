# Profiling

A brief note on profiling Python code.

## Tools

- https://docs.python.org/3/library/profile.html#module-cProfile
- https://github.com/baverman/flameprof
- https://github.com/jlfwong/speedscope

Run `cProfile` and output to `.txt`

```bash
python -m cProfile --sort time ./tests/test_terrain_ompl_rrt.py > ./prof/test_terrain_ompl_rrt.txt
```

Run `cProfile` and output to `.prof`

```bash
cd ./tests
python -m cProfile -o ./prof/test_terrain_ompl_rrt.prof ./tests/test_terrain_ompl_rrt.py
```

Convert output to flameprof log:

```bash
flameprof --format=log ./prof/test_terrain_ompl_rrt.prof > ./prof/test_terrain_ompl_rrt.log
```

Load in [speedscope](https://www.speedscope.app/)

## Optimisation

Use OMPL `OwenStateSpace` (which is the DubinsAirplane model).


Missing interface functions
- Q: why not use the main interpolate function to create the
  trajectory?
- Q: how is the OwenStateSpace model structured internally - similarities
  and differences with the DubinsAirplanStateSpace model.

```c++
void interpolate(const State *from, const State *to, double t, State *state) 
const override;
```

equivalent

```python
def interpolate(
    self, from_state: ob.State, to_state: ob.State, t: float, state: ob.State
) -> None:
```

```c++
virtual void interpolate(const State *from, const State *to, double t, PathType &path, State *state) const;
```

similar

```python
    def interpolate2(
        self,
        from_state: ob.State,
        to_state: ob.State,
        t: float,
        firstTime: float,
        path: DubinsPath,
        segmentStarts: SegmentStarts,
        state: ob.State,
    ) -> None:
```

```c++
std::optional<PathType> getPath(const State *state1, const State *state2) const;
```

equivalent 

```python
def dubins2(self, state1: ob.State, state2: ob.State) -> DubinsPath:
```

### Calculating the SegmentStarts using the OwenStateSpace

The OMPL state space classes do not calculate the detailed path information
required for a path guidance algorithm. In the ETHZ code this is done using
additional methods to capture details of the segments in each Dubins curve.

The method we need to replicate is:

`calculateSegmentStarts`
- The initial state and Dubins path are known.
- DubinsAirplaneStateSpace properties and methods:
  - _gammaMax
  - _tanGammaMax
  - _rho
  - convert_idx()
- DubinsPath properties and methods:
  - length_2d()
  - getGamma()
  - getSegmentLength()
  - getType()
  - getInverseRadiusRatio()
  - getRadiusRatio()

OMPL classes
- OwensStateSpace
- OwensStateSpace.PathType
