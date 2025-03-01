# Drones_club_2024
Repository for working on the aerial autonomous transport project based on the gym-pybullet-drones simulator.


## Run for the first time
- clone repo
- `cd Drones_club_2024`
- `conda create -n drones python=3.10`
- `conda activate drones`
- `sh build.sh`
- `python3 main.py`

If you have problem "libGL error: MESA-LOADER: failed to open swrast:" etc:
- `conda install -c conda-forge libgcc=5.2.0`
- `conda install -c anaconda libstdcxx-ng`
- `conda install -c conda-forge gcc=12.1.0`
