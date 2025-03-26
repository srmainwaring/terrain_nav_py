# Geodesy

The terrain data used in the planner is from the
Shuttle Radar Topography Mission which for our use is easily
available via the `mp_elevation` module in [MAVProxy](https://github.com/ArduPilot/MAVProxy).

## SRTM1 and SRTM30 


| | |
| --- | --- |
|Projection	|Geographic |
|Horizontal Datum	|WGS84 |
|Vertical Datum|	EGM96 (Earth Gravitational Model 1996)|
|Vertical Units|	Meters|
|Spatial Resolution	|1 arc-second for global coverage (~30 meters), 3 arc-seconds for global coverage (~90 meters)|
|Raster Size|	1 degree tiles
|C-band Wavelength|	5.6 cm|

## References

- https://www.usgs.gov/centers/eros/science/usgs-eros-archive-digital-elevation-shuttle-radar-topography-mission-srtm-1
