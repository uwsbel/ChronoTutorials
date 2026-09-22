#  Copyright (c) 2021, SBEL GPU Development Team
#  Copyright (c) 2021, University of Wisconsin - Madison
#  SPDX-License-Identifier: BSD-3-Clause
#
# pyDEME port of DEMdemo_SingleSphereCollide.
# Two spheres approach each other and interact through a CUSTOM cohesion force
# model (ForceModelWithCohesion.cu) that adds an attractive term keyed off a
# pairwise "Cohesion" material property. Also shows owner + per-contact wildcards.
#
# The .cu force-model file is unchanged from the C++ demo and ships with DEME.
# Only the DRIVER (this script) changes between C++ and Python; the physics you
# add lives in the .cu text file and is JIT-compiled at Initialize().
#
# Binding notes (verified against the installed pyDEME):
#  - SetOutputFormat takes a STRING ("CSV"), not an enum.
#  - float3 args are plain Python sequences: a position is [x, y, z], and a
#    per-clump list of positions/velocities is [[x, y, z], ...].
# Env: native Linux (WSL can build but often will not run), CUDA 12.8.

import os
import deme

#0. helper function 
def as_xyz(v):
    """Pos()/vectors may come back as a float3-like object or a sequence."""
    try:
        return v.x, v.y, v.z
    except AttributeError:
        return v[0], v[1], v[2]

#1. DEM output config 
out_dir = os.path.join(os.getcwd(), "DemoOutput_SingleSphereCollide")
os.makedirs(out_dir, exist_ok=True)

DEMSim = deme.DEMSolver()
DEMSim.SetVerbosity("STEP_METRIC")
DEMSim.SetOutputFormat("CSV")
DEMSim.SetContactOutputContent(
    ["OWNER", "FORCE", "POINT", "NORMAL", "TORQUE", "CNT_WILDCARD"]
)
DEMSim.EnsureKernelErrMsgLineNum()


###################################################################
#BLOCK (1): Define System Geometry and Contact Model (Materials)
###################################################################

#1.1 Materials 

# ---------------------------------------------------------------------------
# Materials. The custom "Cohesion" entry is an EXTRA property that the default
# Hertz-Mindlin model ignores but our custom force model reads.
# ---------------------------------------------------------------------------
mat_type_1 = DEMSim.LoadMaterial({"E": 1e9, "nu": 0.3, "CoR": 0.8, "mu": 0.3, "Crr": 0.01, "Cohesion": 50})
mat_type_2 = DEMSim.LoadMaterial({"E": 2e9, "nu": 0.4, "CoR": 0.6, "mu": 0.3, "Crr": 0.01, "Cohesion": 50})
mat_type_3 = DEMSim.Duplicate(mat_type_2)

# Pairwise values take precedence over the per-material values above whenever
# two DIFFERENT materials are in contact.
DEMSim.SetMaterialPropertyPair("CoR", mat_type_1, mat_type_2, 0.6)
DEMSim.SetMaterialPropertyPair("Cohesion", mat_type_1, mat_type_2, 100.0)
DEMSim.SetMaterialPropertyPair("Cohesion", mat_type_1, mat_type_3, 100.0)
DEMSim.SetMaterialPropertyPair("Cohesion", mat_type_2, mat_type_3, 100.0)

#1.2 Loading individual particles 

# ---------------------------------------------------------------------------
# Sphere templates:  LoadSphereType(mass, radius, material)
# ---------------------------------------------------------------------------
sph_type_1 = DEMSim.LoadSphereType(11728.0, 1.0, mat_type_1)
sph_type_2 = DEMSim.Duplicate(sph_type_1)

#NOTE: this demo does not utilize the LoadClumpType API, see magic_2026_dem_load_clump_type.md for more details

###########################################################################
#BLOCK (2): Example specific details that sometimes deviate from the tutorial 
#(loading the force model is in this bloc, see 2.1)
###########################################################################

# ---------------------------------------------------------------------------
# Two spheres, launched toward each other along x.
# Positions and velocities are plain [[x, y, z]] sequences (one per clump).
# ---------------------------------------------------------------------------
sphPos = 1.2
particles1 = DEMSim.AddClumps([sph_type_1], [[-sphPos, 0, 0]])
particles1.SetVel([[1.0, 0, 0]])
particles1.SetFamily(0)
particles1.AddOwnerWildcard("mu_custom", 0.5)      # per-particle (owner) wildcard
particles1.AddOwnerWildcard("some_property", 1.0)  # unused wildcard is harmless
tracker1 = DEMSim.Track(particles1)

# ---------------------------------------------------------------------------
# Bottom plane (mesh boundary). Requires the shipped data file. # (verify)
# If GetDEMEDataFile is not wrapped or the file is not found, comment this block.
# ---------------------------------------------------------------------------
bot_plane = DEMSim.AddWavefrontMeshObject(
    deme.GetDEMEDataFile("mesh/plane_20by20.obj"), mat_type_2
)
bot_plane.SetInitPos([0, 0, -1.25])
bot_plane.SetMass(10000.0)
tracker_mesh = DEMSim.Track(bot_plane)

# ---------------------------------------------------------------------------
# Inspectors: cheap runtime probes. Optional. # (verify) some may be unwrapped
# ---------------------------------------------------------------------------
max_z_finder = DEMSim.CreateInspector("clump_max_z")
max_v_finder = DEMSim.CreateInspector("clump_max_absv")
KE_finder = DEMSim.CreateInspector("clump_kinetic_energy")

#2.1 custom forcem odel 

# ---------------------------------------------------------------------------
# THE CUSTOM FORCE MODEL. Read a .cu text file; it is JIT-compiled at
# Initialize(). We must tell the solver about the state/props the file uses.
# ---------------------------------------------------------------------------
my_force_model = DEMSim.ReadContactForceModel("magic_2026_dem_force_model.cu")
# It still uses the standard tangential-history contact wildcards:
my_force_model.SetPerContactWildcards(
    {"delta_time", "delta_tan_x", "delta_tan_y", "delta_tan_z"}
)
# "Cohesion" (plus the usual pairwise props) is resolved per material-pair:
my_force_model.SetMustPairwiseMatProp({"CoR", "mu", "Crr", "Cohesion"})

# ---------------------------------------------------------------------------
# Solver settings.
# ---------------------------------------------------------------------------
DEMSim.SetInitTimeStep(2e-5)
DEMSim.SetGravitationalAcceleration([0, 0, -9.8])
DEMSim.SetCDUpdateFreq(10)
DEMSim.SetMaxVelocity(6.0)
DEMSim.SetExpandSafetyType("auto")
DEMSim.SetExpandSafetyMultiplier(1.2)
DEMSim.SetIntegrator("centered_difference")

DEMSim.Initialize()

# Clumps can be added AFTER Initialize() in DEME:
particles2 = DEMSim.AddClumps([sph_type_2], [[sphPos, 0, 0]])
particles2.SetVel([[-1.0, 0, 0]])
particles2.SetFamily(1)
tracker2 = DEMSim.Track(particles2)
DEMSim.UpdateClumps()

# ---------------------------------------------------------------------------
# Run + output loop: 1 s total, 100 frames.
# ---------------------------------------------------------------------------
frame_time = 1e-2
n_frames = int(1.0 / frame_time)
for i in range(n_frames):
    print("Frame:", i)

    DEMSim.WriteSphereFile(os.path.join(out_dir, "DEMdemo_output_%04d.csv" % i))
    DEMSim.WriteContactFile(os.path.join(out_dir, "Contact_pairs_%04d.csv" % i))
    DEMSim.WriteMeshFile(os.path.join(out_dir, "DEMdemo_mesh_%04d.vtk" % i))

    DEMSim.DoDynamicsThenSync(frame_time)

    max_z = max_z_finder.GetValue()
    max_v = max_v_finder.GetValue()
    KE = KE_finder.GetValue()
    x1, y1, z1 = as_xyz(tracker1.Pos())
    x2, y2, z2 = as_xyz(tracker2.Pos())

    print("  max Z = %.4f | max |v| = %.4f | KE = %.4f" % (max_z, max_v, KE))
    print("  p1.x  = %.4f | p2.x   = %.4f" % (x1, x2))

DEMSim.ShowThreadCollaborationStats()
DEMSim.ShowTimingStats()
print("DEMdemo_SingleSphereCollide (pyDEME) exiting...")