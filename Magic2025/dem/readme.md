
## Rotating Drum DEM Tutorial

This tutorial demonstrates a 2D rotating drum simulation using PyDEME.

### How to Run

From the `dem` folder, run:

```bash
python pyDEME_RotatingDrum2D.py
```

### Expected Output

You should see temperature increasing at every frame in the terminal. Results may vary slightly due to randomized initial particle positions.

Output files will be saved in the `HeatTransfer` folder and can be visualized with ParaView:
- `DEMdemo_mesh_XXXX.vtk`: Mesh at each output frame
- `DEMdemo_output_XXXX.csv`: Tabular data for all particles per frame

![Terminal Output](terminal_output.png)
![Drum Output](drum.png)
