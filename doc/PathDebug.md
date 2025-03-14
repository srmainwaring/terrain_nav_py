# Path Debug


C++ version

```bash
# dubins(2)
x1 -40
y1 -0
z1 73.918599999999998
th1 1.5708
x2 -187.63929999999999
y2 161.95769999999999
z2 104.6088
th2 -2.8273999999999999
dx -3.6909825000000001
dy 4.0489424999999999
dz 0.76725500000000013
fabs_dz 0.76725500000000013
d 5.4788031578063965
th 2.3099789619445801
alpha 5.5440063452350063
beta 1.1458063452350062
L 5.7927454504596154
gammaMax_ 0.10000000000000001
tanGammaMax_ 0.10033467208545054

[TerrainOmplRrt] dubins.idx:  TYPE_LSL
[TerrainOmplRrt] dubins.type: DUBINS_LEFT DUBINS_STRAIGHT DUBINS_LEFT
[TerrainOmplRrt] dubins.len:  0.655476, 3.90776, 1.22951
[TerrainOmplRrt] dubins.alt:  ALT_CASE_MEDIUM
[TerrainOmplRrt] dubins.cls:  CLASS_A41
[TerrainOmplRrt] segs[0]:     -40.000000, 0.000000, 73.918600, 1.570800, 
[TerrainOmplRrt] segs[0]:     -40.000000, 0.000007, 89.886911, 1.570800, 
[TerrainOmplRrt] segs[0]:     -48.289778, 24.381431, 91.552761, 2.226276, 
[TerrainOmplRrt] segs[0]:     -48.289778, 24.381431, 91.552761, 2.226276, 
[TerrainOmplRrt] segs[0]:     -143.567165, 148.297381, 101.484082, 2.226276, 
[TerrainOmplRrt] segs[0]:     -187.639345, 161.957650, 104.608801, -2.827400,
```

Python version

```bash
# dubins2
x1 -40.0
y1 -0.0
z1 73.9186
th1 1.5708
x2 -187.6393
y2 161.9577
z2 104.6088
th2 -2.8274
dx -3.6909825
dy 4.0489425
dz 0.7672550000000001
fabs_dz 0.7672550000000001
d 5.478803444513455
th 2.3099789118063154
alpha 5.544006395373271
beta 1.1458063953732704
L 5.792745874621915
gammaMax 0.1
tanGammaMax 0.10033467208545054

[TerrainOmplRrt] state[0]:    [-40.0000, -0.0000, 73.9186; 1.5708]
[TerrainOmplRrt] state[1]:    [-187.6393, 161.9577, 104.6088; -2.8274]
[TerrainOmplRrt] dubins.idx:  TYPE_LSL
[TerrainOmplRrt] dubins.type: DUBINS_LEFT DUBINS_STRAIGHT DUBINS_LEFT
[TerrainOmplRrt] dubins.len:  0.655       3.908          1.230
[TerrainOmplRrt] dubins.alt:  ALT_CASE_MEDIUM
[TerrainOmplRrt] dubins.cls:  CLASS_A41
[TerrainOmplRrt] dubins.ks:   1
[TerrainOmplRrt] dubins.ke:   0
[TerrainOmplRrt] segs[0]:     x: -40.000, y: 0.000, z: 73.919, yaw: 1.571
[TerrainOmplRrt] segs[1]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
[TerrainOmplRrt] segs[2]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
[TerrainOmplRrt] segs[3]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
[TerrainOmplRrt] segs[4]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
[TerrainOmplRrt] segs[5]:     x: 0.000, y: 0.000, z: 0.000, yaw: 0.000
```