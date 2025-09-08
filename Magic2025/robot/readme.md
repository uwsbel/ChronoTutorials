
## Robot Simulation Tutorial

This tutorial demonstrates robot simulation on both rigid and CRM granular terrain using PyChrono.

### How to Run

From the `robot` folder, run the following commands:

**0. Install necessary dependency inside your pychrono conda environment (assume your env name is called `tutorial`)**
```bash
conda activate tutorial
pip install matplotlib torch torchvision rsl-rl-lib==2.2.4 tensorboard
```

**1. Test robot on rigid terrain:**
```bash
python simulation/playground.py
```

**2. Test robot on CRM granular terrain:**
```bash
python simulation/playground_crm.py
```

### Expected Output

Results will be similar to the figures below:

![Robot on Rigid Terrain](data/vis-example/demo1.png)
*Figure 1: Robot on rigid terrain simulation*

![Robot on CRM Granular Terrain](data/vis-example/demo2.png)
*Figure 2: Robot on CRM granular terrain simulation*

