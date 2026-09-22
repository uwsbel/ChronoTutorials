> **Note:** This demo does not use the `AddClumps` API. However, since clump creation is relevant to this tutorial, the basic `LoadClumpType` syntax is shown below.

### `LoadClumpType`

`LoadClumpType` defines a reusable **clump type/template** for the DEM simulation. It specifies the clump's mass, rotational inertia, geometry, and contact material. The returned `ellipsoid` object can later be used to instantiate clumps in the simulation.

```python
ellipsoid = solver.LoadClumpType(
    mass,                                              # Total mass of one clump
    [mass, mass, 0.4 * mass],                        # Principal moments of inertia [Ixx, Iyy, Izz]
    deme.GetDEMEDataFile("clumps/ellipsoid_2_1_1.csv"), # File defining the clump geometry
    sand                                               # Contact material assigned to the clump
)
```

#### Arguments

- `mass`  
  The total mass of one clump.

- `[mass, mass, 0.4 * mass]`  
  The principal moments of inertia, `[Ixx, Iyy, Izz]`, which describe the clump's resistance to rotation about each principal axis.

- `deme.GetDEMEDataFile("clumps/ellipsoid_2_1_1.csv")`  
  Retrieves the path to the CSV file that defines the clump geometry, including the positions and sizes of the component spheres.

- `sand`  
  The DEM material assigned to the clump. It defines the contact properties used when the clump interacts with other objects.