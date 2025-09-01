
## Sensor Tutorial

This tutorial demonstrates the use of sensors in PyChrono.

### Setup

Copy the `boulder_rock` folder into your PyChrono environment's sensor data directory:

- **Windows:**
	```
	[your_pychrono_env]\Library\data\sensor\offroad\
	```
- **Linux:**
	```bash
	[your_pychrono_env]/share/chrono/data/sensor/offroad/
	```

Replace `[your_pychrono_env]` with the root directory of your PyChrono environment. In order to find out the the root directory, use this command:

```bash
conda info --envs
```
This lists the Conda environments on your machine and their root directories. The one with the `*` next to it is your active environment, which provides the information regarding the `[your_pychrono_env]` root directory that you need. 

### How to Run

From withing the `sensor` folder, run:
```bash
python script/tutorial_SEN_camera.py
```

### Expected Output

The script will run a sensor simulation and output results using the provided boulder rock data.
