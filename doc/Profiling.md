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


