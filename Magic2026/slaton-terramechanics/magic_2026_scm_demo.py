"""
PyChrono Tutorial - Deformable Terrain with the SCM (Soil Contact Model)

WHAT YOU'LL LEARN
How Chrono's SCMTerrain turns ground into deformable soil that a vehicle sinks
into and carves ruts in. You'll configure the soil physics, the moving compute
region, the terrain geometry, the sinkage coloring, and wire the terrain into
the simulation loop.

WHO THIS IS FOR
You know Python but are new to Chrono. You do NOT need to understand the HMMWV
vehicle or the path-following driver - those sections are marked GIVEN. Every
EXERCISE is about the SCM terrain only.

THIS IS THE EXERCISE FILE
The SCM calls are blanked out. Run the file and it stops at the first
unfinished exercise with a message telling you what to add and where. Write
that call, delete the raise line, and run again to advance to the next one.
The answers live in magic_2026_scm_solutions.py - try not to peek until
you've had a go.

HOW TO RUN
Needs PyChrono with the vehicle and vsg modules, plus numpy:
    python scm_terrain_tutorial.py              # do the exercises
    python scm_terrain_tutorial.py --solutions  # just watch the finished demo
A 3D window opens - that's where you watch the soil deform. The --solutions
flag runs magic_2026_scm_solutions.py, so it works even before you've
filled anything in.

THE EXERCISES (all in the SCM sections #8 and #11)
1. Soil parameters - terrain.SetSoilParameters(), the ground's physics
2. Active domain - terrain.AddActiveDomain(), the moving compute box
3. Sinkage plot - terrain.SetPlotType(), color the soil by depth
4. Terrain geometry - terrain.Initialize(), build the patch from the heightmap
5. Loop integration - terrain.Synchronize()/Advance(), make the ruts persist
"""

import os
import sys

# Shortcut: `python scm_terrain_tutorial.py --solutions` runs the completed
# reference demo (magic_2026_scm_solutions.py) without you filling in
# anything - handy for seeing the finished result first. The answers live in
# that separate file, not here, so this file's source stays blank.
if "--solutions" in sys.argv:
    import runpy
    _here = os.path.dirname(os.path.abspath(__file__))
    runpy.run_path(os.path.join(_here, "magic_2026_scm_solutions.py"), run_name="__main__")
    sys.exit(0)

import struct
import tempfile

import numpy as np

import pychrono as chrono
import pychrono.vehicle as veh


############################################################################
# GIVEN
# SCM TERRAIN GENERATION HELPERS
# description: make bumpy/rough SCM terrain via a heightmap
############################################################################

def _write_grayscale_bmp(path, heightfield):
    """Write a 2D uint8 array as an uncompressed 24-bit grayscale BMP."""
    h, w = heightfield.shape
    row_size = (w * 3 + 3) & ~3
    pixel_data_size = row_size * h
    file_size = 54 + pixel_data_size

    with open(path, "wb") as f:
        f.write(b"BM")
        f.write(struct.pack("<IHHI", file_size, 0, 0, 54))
        f.write(struct.pack("<IiiHHIIiiII", 40, w, h, 1, 24, 0, pixel_data_size, 2835, 2835, 0, 0))
        padding = b"\x00" * (row_size - w * 3)
        for row in heightfield[::-1]:  # BMP rows are stored bottom-up
            f.write(np.repeat(row, 3).astype(np.uint8).tobytes())
            f.write(padding)


def _generate_rough_road_heightmap(path, length, width, resolution, num_waves, seed):
    """Procedurally build a band-limited bumpy heightfield and save it as a BMP.

    Sums random-direction sine waves with amplitude weighted by wavelength (longer
    wavelength -> larger amplitude), which gives rolling, road-like bumps instead of
    uniform washboard ripples or unrealistic single-pixel noise.
    """
    rng = np.random.default_rng(seed)
    xs = np.linspace(0, length, resolution)
    ys = np.linspace(0, width, resolution)
    x, y = np.meshgrid(xs, ys)

    field = np.zeros_like(x)
    for _ in range(num_waves):
        wavelength = rng.uniform(0.5, 4.0)
        angle = rng.uniform(0, 2 * np.pi)
        phase = rng.uniform(0, 2 * np.pi)
        k = 2 * np.pi / wavelength
        field += wavelength * np.sin(k * (x * np.cos(angle) + y * np.sin(angle)) + phase)

    field -= field.min()
    field /= field.max()
    _write_grayscale_bmp(path, (field * 255).astype(np.uint8))


############################################################################
# PYCHRONO SIMULATION CODE
# description: simulation code for a HMMWV vehicle driving on a rough road
############################################################################

# GIVEN
#0. simulation parameters
TIME_STEP = 1e-3
TOTAL_SIM_TIME = 10.0

# GIVEN
#1. vehicle config parameters
contact_method = chrono.ChContactMethod_SMC
chassis_collision_type = veh.CollisionType_NONE
chassis_fixed = False
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)
engine_type = veh.EngineModelType_SHAFTS
transmission_type = veh.TransmissionModelType_AUTOMATIC_SHAFTS
drive_type = veh.DrivelineTypeWV_AWD
steering_type = veh.SteeringTypeWV_PITMAN_ARM
tire_type = veh.TireModelType_RIGID
tire_step_size = 1e-3

chassis_vis_type = chrono.VisualizationType_MESH
suspension_vis_type = chrono.VisualizationType_PRIMITIVES
steering_vis_type = chrono.VisualizationType_PRIMITIVES
wheel_vis_type = chrono.VisualizationType_MESH
tire_vis_type = chrono.VisualizationType_MESH

#2. terrain config parameters
# These 8 values feed terrain.SetSoilParameters() in Exercise 1 (section #8).
# What each one controls:
# bekker_kphi (N/m^(n+2)): pressure-vs-sinkage stiffness, friction part
# bekker_kc (N/m^(n+1)): pressure-vs-sinkage stiffness, cohesion part
# bekker_n_exp: pressure-vs-sinkage curve exponent
# mohr_cohesive_limit (Pa): shear strength at zero normal load
# mohr_friction_limit (deg): internal friction angle of the soil
# janosi_shear_coeff (m): shear displacement needed for full traction
# elastic_stiffness (Pa/m): elastic rebound before plastic yield (keep > kphi)
# damping (Pa*s/m): vertical damping proportional to sinkage rate (stability)
# Experiment later: set damping to 0 then 3e5, and kphi to 0.2e6 then 5e6,
# and watch how deep the tires sink and how jittery the contact looks.
bekker_kphi = 2e6
bekker_kc = 0.0
bekker_n_exp = 1.1
mohr_cohesive_limit = 0.0
mohr_friction_limit = 30.0
janosi_shear_coeff = 0.01
elastic_stiffness = 2e8
damping = 3e4
terrain_params = (bekker_kphi, bekker_kc, bekker_n_exp, mohr_cohesive_limit, mohr_friction_limit, janosi_shear_coeff, elastic_stiffness, damping)
terrain_length = 200.0
terrain_width = 20.0
terrain_delta = 0.05

#3. heightmap generation parameters
terrain_height_min = -0.15
terrain_height_max = 0.15
heightmap_resolution = 513
heightmap_num_waves = 20
heightmap_seed = 1

# GIVEN
#4. chase camera parameters
track_point = chrono.ChVector3d(0.0, 0.0, 1.75)
chase_distance = 10.0
chase_height = 1.5

# GIVEN
#5. path-follower waypoints: a gentle winding line down the road strip,
path_points = [
    chrono.ChVector3d(0, 0, 0.5),
    chrono.ChVector3d(20, 2, 0.5),
    chrono.ChVector3d(40, -2, 0.5),
    chrono.ChVector3d(60, 3, 0.5),
    chrono.ChVector3d(80, -3, 0.5),
    chrono.ChVector3d(90, 0, 0.5),
]

# GIVEN
#6. path-follower controller parameters
target_speed = 8.0
steering_lookahead_distance = 5.0
steering_gains = (0.8, 0, 0)
speed_gains = (0.4, 0, 0)

# GIVEN
#7. create the vehicle and its visual system
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(chassis_fixed)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetEngineType(engine_type)
vehicle.SetTransmissionType(transmission_type)
vehicle.SetDriveType(drive_type)
vehicle.SetSteeringType(steering_type)
vehicle.SetTireType(tire_type)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chassis_vis_type)
vehicle.SetSuspensionVisualizationType(suspension_vis_type)
vehicle.SetSteeringVisualizationType(steering_vis_type)
vehicle.SetWheelVisualizationType(wheel_vis_type)
vehicle.SetTireVisualizationType(tire_vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

#8. create the terrain and initialize it from a procedurally generated heightmap
heightmap_file = tempfile.NamedTemporaryFile(suffix=".bmp", delete=False).name
try:
    # GIVEN
    # paint the bumpy heightfield to a temp BMP
    _generate_rough_road_heightmap(heightmap_file, terrain_length, terrain_width,
                                    heightmap_resolution, heightmap_num_waves, heightmap_seed)

    # GIVEN
    # create the (empty) SCM terrain, bound to the vehicle's physical system
    terrain = veh.SCMTerrain(vehicle.GetSystem())

    # EXERCISE 1 - soil parameters
    # Configure the soil physics. This is the single most important SCM call:
    # it decides how hard the ground pushes back. Call terrain.SetSoilParameters()
    # and pass the 8 values from section #2 (packed for you as terrain_params).
    # See the table in #2 for what each argument controls.
    # TODO: write the SetSoilParameters call, then delete the line below.
    raise NotImplementedError("EXERCISE 1: call terrain.SetSoilParameters(*terrain_params)")

    # EXERCISE 2 - active domain
    # SCM would be far too slow if it deformed the whole 200 m patch every step,
    # so it only updates a box that follows a body. Attach one to the chassis:
    # terrain.AddActiveDomain(body, offset, size)
    # body   = vehicle.GetChassisBody()
    # offset = chrono.ChVector3d(0, 0, 0)   -> centered on the chassis
    # size   = chrono.ChVector3d(5, 3, 1)   -> length, width, height in metres
    # TODO: add the active domain, then delete the line below.
    raise NotImplementedError("EXERCISE 2: call terrain.AddActiveDomain(...)")

    # EXERCISE 3 - sinkage visualization
    # This is what makes SCM visible: color each soil node by how deep it sank.
    # terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
    # The last two args are the color-scale range in metres (0 = undisturbed,
    # 0.1 = deepest color). Try PLOT_PRESSURE instead of PLOT_SINKAGE later.
    # TODO: set the plot type, then delete the line below.
    raise NotImplementedError("EXERCISE 3: call terrain.SetPlotType(...)")

    # EXERCISE 4 - build the terrain geometry
    # Turn the heightmap BMP into an actual deformable patch:
    # terrain.Initialize(heightmap_file, terrain_length, terrain_width,
    #                    terrain_height_min, terrain_height_max, terrain_delta)
    # terrain_delta is the grid spacing: smaller = finer detail but slower.
    # TODO: initialize the terrain, then delete the line below.
    raise NotImplementedError("EXERCISE 4: call terrain.Initialize(...)")
finally:
    os.remove(heightmap_file)

# GIVEN
#9. create the path-following driver: a Bezier curve through the waypoints
path = chrono.ChBezierCurve(chrono.vector_ChVector3d(path_points), False)
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, "path", target_speed)
driver.GetSteeringController().SetLookAheadDistance(steering_lookahead_distance)
driver.GetSteeringController().SetGains(*steering_gains)
driver.GetSpeedController().SetGains(*speed_gains)
driver.Initialize()

# GIVEN
#10. create the chase camera that tracks the vehicle
visual = veh.ChWheeledVehicleVisualSystemVSG()
visual.AttachVehicle(vehicle.GetVehicle())
visual.SetWindowTitle("Vehicle Rough Road")
visual.SetWindowSize(1200, 800)
visual.SetCameraVertical(chrono.CameraVerticalDir_Z)
visual.SetChaseCamera(track_point, chase_distance, chase_height)
visual.Initialize()

#11. sim loop
sys = vehicle.GetSystem()
while sys.GetChTime() < TOTAL_SIM_TIME and visual.Run():
    time = sys.GetChTime()
    driver_inputs = driver.GetInputs()

    # GIVEN
    # synchronize driver, vehicle, and visuals to the current time
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    visual.Synchronize(time, driver_inputs)

    # EXERCISE 5 (capstone) - put SCM in the loop, part 1 of 2
    # The terrain is a co-simulated subsystem: it must be synchronized to the
    # current time here, and advanced by one step further down. Skip these and
    # the vehicle drives but the ground never deforms and no ruts appear.
    # TODO: synchronize the terrain to `time`, then delete the line below.
    raise NotImplementedError("EXERCISE 5a: call terrain.Synchronize(time)")

    visual.Render()

    # GIVEN
    # advance driver, vehicle, and visuals by one time step
    driver.Advance(TIME_STEP)
    vehicle.Advance(TIME_STEP)
    visual.Advance(TIME_STEP)

    # EXERCISE 5 (capstone) - part 2 of 2
    # TODO: advance the terrain by TIME_STEP, then delete the line below.
    raise NotImplementedError("EXERCISE 5b: call terrain.Advance(TIME_STEP)")