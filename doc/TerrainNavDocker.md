  # Terran Navigation Docker

```bash
# ompl wheels
https://github.com/ompl/ompl/releases/download/prerelease
```

## Set timezone

```bash
RUN ln -snf /usr/share/zoneinfo/$CONTAINER_TIMEZONE /etc/localtime && echo $CONTAINER_TIMEZONE > /etc/timezone
```

For docker - set non-interactive 
```bash
ARG DEBIAN_FRONTEND=noninteractive
```

## Ubuntu 22.04, aarch64

```bash
# on host
docker pull ubuntu:22.04
docker run -it --rm ubuntu:22.04

# on docker container
export DEBIAN_FRONTEND=noninteractive
export TZ=Etc/UTC
apt update
apt install git -y
apt install python3 -y
apt install python3-dev -y
apt install python3-venv -y
python3 -m venv --system-site-packages venv
. venv/bin/activate
python3 -m pip install --upgrade pip
git clone https://github.com/srmainwaring/terrain_nav_py.git
cd terrain_nav_py/
python -m pip install .
pytest .
```

Install `terrain_nav_py` directly.

```bash
python -m pip install git+https://github.com/srmainwaring/terrain_nav_py.git
```
