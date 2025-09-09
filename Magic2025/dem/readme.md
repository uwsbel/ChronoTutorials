## Rotating Drum DEM Tutorial

This tutorial demonstrates a 2D rotating drum simulation using PyDEME.

### How to Run

From the `dem` folder, run:

```bash
python pyDEME_RotatingDrum2D.py
```


### How to Enable IntelliSense and Autocomplete

To enable code completion and documentation for PyDEME in VS Code, make sure the `DEME.pyi` stub file (included in the `dem` folder) is present in your Python environment:

- **Linux:**
	Copy `DEME.pyi` to:
	```
	[your_conda_env]/lib/python3.12/site-packages/DEME/
	```
- **Windows:**
	The typical path is:
	```
	[your_conda_env]\Lib\site-packages\DEME
	```
To find the root directory of your Python environment, use this command,

```bash
conda info --envs
```
This will list all Conda environments and their root directories. The environment with the `*` next to it is your active environment, which provides the path you need for copying files like `DEME.pyi`. After copying, reload VS Code or the Python extension to refresh autocomplete features.

### Installing ParaView
To install ParaView for visualization, simply run:
```bash
conda install paraview
```
This will install ParaView in your current conda environment.


### Expected Output

You should see temperature increasing at every frame in the terminal. Results may vary slightly due to randomized initial particle positions.


Output files will be saved in the `HeatTransfer` folder and can be visualized with ParaView:
- `DEMdemo_mesh_XXXX.vtk`: Mesh at each output frame
- `DEMdemo_output_XXXX.csv`: Tabular data for all particles per frame

![Terminal Output](terminal_output.png)
![Drum Output](drum.png)
