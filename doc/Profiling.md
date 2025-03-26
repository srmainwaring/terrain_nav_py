# Profiling

Tools

- https://github.com/baverman/flameprof
- https://github.com/jlfwong/speedscope

Run cProfiler: output to `.txt`

```bash
python -m cProfile --sort time ./test_terrain_ompl_rrt.py > ../prof/test_terrain_ompl_rrt.txt
```

Output to `.prof`

```bash
cd ./tests
python -m cProfile -o ../prof/test_terrain_ompl_rrt.prof ./test_terrain_ompl_rrt.py
```

Convert output to flameprof

```bash
flameprof --format=log ../prof/test_terrain_ompl_rrt.prof > ../prof/test_terrain_ompl_rrt.log
```

Load in [speedscope](https://www.speedscope.app/)


Line Profiler

- https://kernprof.readthedocs.io/en/latest/

