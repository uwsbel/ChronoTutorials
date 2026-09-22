"""
MAGIC 2026 - CRM Normal Bevameter (pressure-sinkage) Test

WHAT THIS IS
A normal bevameter test: a rigid circular plate is driven straight down into a
bed of CRM (SPH continuum) soil at a constant rate, and we record the soil
reaction force vs how far the plate has sunk. That reaction-pressure-vs-sinkage
curve is the classic terramechanics measurement.

WHO THIS IS FOR
You know Python but are new to Chrono. The Chrono systems, the SPH solver
settings, the container walls, the plate body, and the visualization are all
marked GIVEN. Every EXERCISE is about the CRM/FSI part.

THIS IS THE EXERCISE FILE
The CRM/FSI calls are blanked out. Run the file and it stops at the first
unfinished exercise with a message telling you what to add and where. Write that
code, delete the raise line, and run again to advance. The answers live in
magic_2026_crm_solutions.py - try not to peek until you've had a go. (If you
rename this file, update the solution filename in the --solutions block below.)

TWO FSI GOTCHAS ALREADY HANDLED FOR YOU (both in GIVEN sections):
1. Order: the SPH soil particles are created BEFORE SetComputationalDomain and
   before the container/BCE. Do it in the other order and the terrain comes out
   wrong.
2. AddFsiBoundary ignores its frame argument in PyChrono, so the container BCE
   markers are shifted into place by hand before being added.

HOW TO RUN
Needs an NVIDIA (CUDA) GPU and a Chrono build with the FSI module, plus the
pychrono fsi (and vsg3d for the window) modules. Then:
    python magic_2026_crm_demo.py              # do the exercises
    python magic_2026_crm_demo.py --solutions  # watch the finished demo
At the end a reaction-pressure-vs-sinkage curve is written to CSV (and plotted
if matplotlib is installed).

THE EXERCISES (all CRM/FSI)
1. Soil properties - ElasticMaterialProperties + SetElasticSPH()
2. Create the soil bed - AddSPHParticle() over the sampled grid, geostatic stress
3. Couple the plate - CreatePointsCylinderInterior() + AddFsiBody()
4. Drive the plate - ChLinkMotorLinearPosition at a constant rate
5. Step & measure - DoStepDynamics(); reaction force via GetFsiBodyForce()
"""

import os
import sys

# Shortcut: `python magic_2026_crm_demo.py --solutions` runs the completed demo.
# Keep this filename in sync with the solution file if you rename either one.
if "--solutions" in sys.argv:
    import runpy
    _here = os.path.dirname(os.path.abspath(__file__))
    runpy.run_path(os.path.join(_here, "magic_2026_crm_solutions.py"), run_name="__main__")
    sys.exit(0)

import math

import pychrono as chrono
import pychrono.fsi as fsi

try:
    import pychrono.vsg3d as vsg3d
    HAVE_VSG = True
except Exception:
    HAVE_VSG = False


# GIVEN
#0. simulation + geometry parameters (demo-scale so it runs on a normal GPU)
TIME_STEP = 2e-4
container_x = 0.5         # m
container_y = 0.5         # m
bed_depth = 0.25          # m (soil height)
clearance = 0.05 * bed_depth
container_z = bed_depth + clearance
spacing = 0.02            # SPH particle spacing, m
plate_diameter = 0.15     # m
plate_thickness = 0.05    # m
plate_density = 7800.0    # kg/m^3 (steel)
plate_velocity = 0.05     # m/s downward
target_sinkage = 0.05     # m, stop once the plate has sunk this far
render = True
render_fps = 120
log_fps = 500
plate_radius = plate_diameter / 2
plate_area = math.pi * plate_radius ** 2

#1. soil parameters (feed ElasticMaterialProperties in Exercise 1)
density = 1670.0
youngs_modulus = 1e6
poisson_ratio = 0.3
mu_s = 0.6593
mu_2 = 0.6593
cohesion = 0.0

# GIVEN
#2. create the multibody system, the SPH fluid system, and the FSI coupler
sys_mbs = chrono.ChSystemSMC()
sys_sph = fsi.ChFsiFluidSystemSPH()
sys_fsi = fsi.ChFsiSystemSPH(sys_mbs, sys_sph)
sys_fsi.SetStepSizeCFD(TIME_STEP)
sys_fsi.SetStepsizeMBD(TIME_STEP)

# EXERCISE 1 - soil properties
# Define the CRM soil's SPH physics with the mu(I) rheology. Build an
# ElasticMaterialProperties, fill it from the values in #1, and hand it to the
# fluid system:
#   mat_props = fsi.ElasticMaterialProperties()
#   mat_props.density = density
#   mat_props.Young_modulus = youngs_modulus
#   mat_props.Poisson_ratio = poisson_ratio
#   mat_props.rheology_model = fsi.RheologyCRM_MU_OF_I
#   mat_props.mu_I0 = 0.04
#   mat_props.mu_fric_s = mu_s
#   mat_props.mu_fric_2 = mu_2
#   mat_props.average_diam = 0.0002
#   mat_props.cohesion_coeff = cohesion
#   sys_sph.SetElasticSPH(mat_props)
# (On current Chrono main this is fsi.SoilProperties + sys_sph.SetCrmSPH.)
# TODO: write those lines, then delete the line below.
raise NotImplementedError("EXERCISE 1: build ElasticMaterialProperties and call sys_sph.SetElasticSPH(mat_props)")

# GIVEN
#3. SPH solver / numerical settings (safe defaults - you don't tune these)
sph_params = fsi.SPHParameters()
sph_params.integration_scheme = fsi.IntegrationScheme_RK2
sph_params.initial_spacing = spacing
sph_params.d0_multiplier = 1.3
sph_params.artificial_viscosity = 0.5
sph_params.shifting_method = fsi.ShiftingMethod_PPST_XSPH
sph_params.shifting_xsph_eps = 0.5
sph_params.shifting_ppst_pull = 1.0
sph_params.shifting_ppst_push = 3.0
sph_params.free_surface_threshold = 2.0
sph_params.num_proximity_search_steps = 1
sph_params.kernel_type = fsi.KernelType_CUBIC_SPLINE
sph_params.boundary_method = fsi.BoundaryMethod_ADAMI
sph_params.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_BILATERAL
sph_params.use_variable_time_step = False
sys_sph.SetSPHParameters(sph_params)

# GIVEN
#4. gravity (set after the SPH params, as the working script does)
sys_sph.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys_mbs.SetGravitationalAcceleration(sys_sph.GetGravitationalAcceleration())
sys_fsi.SetVerbose(False)

# GIVEN
#5. sample a regular grid of points filling the soil bed, and note the surface.
# ORDER MATTERS: the particles are created next, BEFORE the computational domain
# and the container are set up.
sampler = chrono.ChGridSamplerd(spacing)
box_center = chrono.ChVector3d(0, 0, bed_depth / 2)
box_half_dim = chrono.ChVector3d(container_x / 2 - spacing,
                                 container_y / 2 - spacing,
                                 bed_depth / 2 - spacing)
points = sampler.SampleBox(box_center, box_half_dim)
soil_top_init = max(p.z for p in points)   # initial soil surface height
print(f"soil surface z = {soil_top_init:.4f}, sampled {len(points)} points")

# EXERCISE 2 - create the soil bed
# Turn each sampled point into an SPH particle, initialized with the geostatic
# (overburden) pressure at its depth so the bed starts in equilibrium instead of
# slumping. The grid points and helpers are set up for you; write the loop.
# AddSPHParticle args:
#   (pos, density, pressure, viscosity, velocity, tau_diag, tau_offdiag, consolidation)
# For each point p:
#   pre = rho0 * gz * (bed_depth - p.z)
#   sys_sph.AddSPHParticle(
#       p, rho0, pre, mu0,
#       chrono.ChVector3d(0, 0, 0),
#       chrono.ChVector3d(-pre, -pre, -pre),
#       chrono.ChVector3d(0, 0, 0),
#       pre)
gz = abs(sys_sph.GetGravitationalAcceleration().z)
rho0 = sys_sph.GetDensity()
mu0 = sys_sph.GetViscosity()
# TODO: loop over `points` and AddSPHParticle each, then delete the line below.
raise NotImplementedError("EXERCISE 2: loop over `points` and call sys_sph.AddSPHParticle(...)")

# GIVEN
#6. now that the particles exist, tell the SPH solver the bounds of its world
min_corner = chrono.ChVector3d(-container_x / 2 * 1.2, -container_y / 2 * 1.2, -container_z * 1.2)
max_corner = chrono.ChVector3d(container_x / 2 * 1.2, container_y / 2 * 1.2,
                               (container_z + 0.05 + spacing) * 1.2)
sys_sph.SetComputationalDomain(chrono.ChAABB(min_corner, max_corner), fsi.BC_NONE)

# GIVEN
#7. container: a fixed body, a (disabled) collision box for looks, and the BCE
# walls the soil actually feels. NOTE the manual shift: AddFsiBoundary ignores
# its frame arg in PyChrono, so we move the wall markers into place ourselves.
box = chrono.ChBody()
box.SetPos(chrono.ChVector3d(0, 0, bed_depth / 2))
box.SetFixed(True)
sys_mbs.AddBody(box)

wall_mat = chrono.ChContactMaterialSMC()
wall_mat.SetYoungModulus(193e9)
wall_mat.SetFriction(0.7)
wall_mat.SetRestitution(0.05)
wall_mat.SetAdhesion(0)
chrono.AddBoxContainer(
    box, wall_mat,
    chrono.ChFramed(chrono.ChVector3d(0, 0, container_z / 2), chrono.QUNIT),
    chrono.ChVector3d(container_x, container_y, container_z),
    0.1, chrono.ChVector3i(2, 2, -1), False,
)
box.EnableCollision(False)

box_bce = sys_sph.CreatePointsBoxContainer(
    chrono.ChVector3d(container_x, container_y, container_z),
    chrono.ChVector3i(2, 2, -1),
)
shifted_bce = [chrono.ChVector3d(p.x, p.y, p.z + container_z / 2) for p in box_bce]
sys_fsi.AddFsiBoundary(shifted_bce, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

# GIVEN
#8. create the plate body (a short cylinder) just above the soil surface
BUFFER = 0.01
plate = chrono.ChBody()
plate_z0 = soil_top_init + plate_thickness / 2 + BUFFER
plate.SetPos(chrono.ChVector3d(0, 0, plate_z0))
plate.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
plate_mass = plate_density * plate_area * plate_thickness
plate.SetMass(plate_mass)
_I = plate_mass * plate_radius ** 2 / 2
plate.SetInertiaXX(chrono.ChVector3d(_I, _I, _I))
sys_mbs.AddBody(plate)
chrono.AddCylinderGeometry(plate, wall_mat, plate_radius, plate_thickness,
                           chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0),
                           True, chrono.ChVisualMaterial())

# EXERCISE 3 - couple the plate to the SPH soil
# In CRM the soil is fluid particles, so the plate only interacts once you fill
# it with boundary (BCE) markers and register it as an FSI body. Do:
#   plate_bce = sys_sph.CreatePointsCylinderInterior(plate_radius, plate_thickness, True)
#   sys_fsi.AddFsiBody(plate, plate_bce, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), False)
# (On current Chrono main the second call is sys_fsi.AddRigidBody.)
# TODO: couple the plate, then delete the line below.
raise NotImplementedError("EXERCISE 3: create plate BCE markers and call sys_fsi.AddFsiBody(...)")

# EXERCISE 4 - drive the plate
# Drive the plate straight down at a constant rate with a linear position motor
# between the plate and the fixed container (position = -velocity * t). Do:
#   motor = chrono.ChLinkMotorLinearPosition()
#   motor.SetMotorFunction(chrono.ChFunctionRamp(0.0, -plate_velocity))
#   motor.Initialize(plate, box, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
#   sys_mbs.AddLink(motor)
# TODO: create and add the motor, then delete the line below.
raise NotImplementedError("EXERCISE 4: add a ChLinkMotorLinearPosition that drives the plate down")

# GIVEN
#9. finalize the FSI system, then build the visualization
sys_sph.SetOutputLevel(getattr(fsi, "OutputLevel_CRM_FULL", getattr(fsi, "OutputLevel_STATE", 0)))
sys_fsi.Initialize()

vis = None
if render and HAVE_VSG:
    visFSI = fsi.ChSphVisualizationVSG(sys_fsi)
    visFSI.EnableFluidMarkers(True)
    visFSI.EnableBoundaryMarkers(True)
    visFSI.EnableRigidBodyMarkers(True)
    visFSI.SetSPHColorCallback(fsi.ParticleHeightColorCallback(0.0, bed_depth), chrono.ChColormap.Type_BROWN)
    vis = vsg3d.ChVisualSystemVSG()
    vis.AttachPlugin(visFSI)
    vis.AttachSystem(sys_mbs)
    vis.SetWindowTitle("MAGIC 2026 - CRM Bevameter")
    vis.SetWindowSize(1280, 720)
    vis.AddCamera(chrono.ChVector3d(0, -0.9, 0.5), chrono.ChVector3d(0, 0, 0.12))
    vis.SetLightIntensity(0.9)
    vis.Initialize()

#10. sim loop: press the plate until it reaches the target sinkage
dt = sys_fsi.GetStepSizeCFD()
log_every = max(1, int(round(1.0 / (log_fps * dt))))
time = 0.0
step = 0
render_frame = 0
sinkage = 0.0
history = []   # (sinkage, reaction_pressure)
print("running...")
while sinkage <= target_sinkage:
    # GIVEN
    # pace the render window
    if render and vis is not None and time >= render_frame / render_fps:
        if not vis.Run():
            break
        vis.Render()
        render_frame += 1

    # EXERCISE 5 (capstone) - advance the coupled SPH + multibody problem
    # One call steps the fluid and the plate/motor together by dt. This is the
    # CRM analog of SCM's terrain.Advance(); there is no separate MBS step.
    #   sys_fsi.DoStepDynamics(dt)
    # TODO: advance the coupled dynamics, then delete the line below.
    raise NotImplementedError("EXERCISE 5: call sys_fsi.DoStepDynamics(dt)")

    time += dt
    step += 1

    # GIVEN
    # measure: sinkage from the plate, reaction pressure from the FSI force on it
    plate_bottom = plate.GetPos().z - plate_thickness / 2
    sinkage = soil_top_init - plate_bottom
    if step % log_every == 0:
        reaction_z = sys_fsi.GetFsiBodyForce(0).z   # soil force on FSI body 0 (the plate)
        pressure = abs(reaction_z) / plate_area
        history.append((sinkage, pressure))
        print(f"t={time:.3f}s  sinkage={sinkage * 1000:6.1f} mm  pressure={pressure / 1000:6.2f} kPa")

# GIVEN
# save and (optionally) plot the pressure-sinkage curve
with open("bevameter_pressure_sinkage.csv", "w") as f:
    f.write("sinkage_m,pressure_Pa\n")
    for s, p in history:
        f.write(f"{s:.6f},{p:.3f}\n")
print(f"wrote bevameter_pressure_sinkage.csv ({len(history)} rows)")

try:
    import matplotlib.pyplot as plt
    s = [row[0] * 1000 for row in history if row[0] > 0]   # mm, contact onward
    p = [row[1] / 1000 for row in history if row[0] > 0]   # kPa
    plt.figure()
    plt.plot(s, p, lw=2)
    plt.xlabel("sinkage (mm)")
    plt.ylabel("reaction pressure (kPa)")
    plt.title("Normal bevameter: pressure vs sinkage")
    plt.grid(True)
    plt.savefig("bevameter_pressure_sinkage.png", dpi=120)
    print("saved bevameter_pressure_sinkage.png")
except Exception as e:
    print(f"(plot skipped: {e})")