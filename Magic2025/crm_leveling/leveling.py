# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2023 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Author: Harry Zhang, Ganesh Arivoli, Huzaifa Unjhawala, Radu Serban
# =============================================================================
#
# Gator vehicle on CRM terrain formed by a custom height map
#
# =============================================================================

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.fsi as fsi
import pychrono.vsg as vsg
import pychrono.pardisomkl as mkl
import os
import sys
import math
import argparse
import csv

from enum import Enum

# Vehicle movement state
class VehicleState(Enum):
    FORWARD_TO_POSITIVE = 1
    WAIT_AT_POSITIVE = 2
    BACKWARD_TO_NEGATIVE = 3
    WAIT_AT_NEGATIVE = 4
    FORWARD_TO_POSITIVE_AGAIN = 5

def CreateFSIWheels(vehicle, terrain):
    """
    Add vehicle wheels as FSI solids to the CRM terrain.
    
    Args:
        vehicle: Gator vehicle object
        terrain: CRMTerrain object
    """
    mesh_filename = veh.GetDataFile("gator/gator_tireF_coarse.obj")
    print(f"mesh_filename: {mesh_filename}")
    
    # Create geometry for rigid wheels
    geometry = chrono.ChBodyGeometry()
    # Comment: TrimeshShape constructor may need adjustment based on Python bindings
    geometry.coll_meshes.append(chrono.TrimeshShape(chrono.VNULL, chrono.QUNIT, mesh_filename, chrono.VNULL))
    
    # Iterate through all axles and wheels
    for axle in vehicle.GetVehicle().GetAxles():
        for wheel in axle.GetWheels():
            tire = wheel.GetTire()
            # Check if this is a deformable tire (FEA)
            try:
                # Try to get FEA mesh if it's a deformable tire
                if hasattr(tire, 'GetMesh'):
                    mesh = tire.GetMesh()
                    if mesh and mesh.GetNumContactSurfaces() > 0:
                        surf = mesh.GetContactSurface(0)
                        print("FEA tire HAS contact surface")
                        # Add FEA mesh to terrain
                        terrain.AddFeaMesh(mesh, False)
                    else:
                        print("FEA tire DOES NOT HAVE contact surface!")
                        terrain.AddFeaMesh(mesh, False)
                else:
                    # Rigid tire - add as rigid body
                    terrain.AddRigidBody(wheel.GetSpindle(), geometry, False)
            except Exception as e:
                print(f"Error processing wheel: {e}")
                # If we can't access FEA mesh methods, treat as rigid tire
                try:
                    terrain.AddRigidBody(wheel.GetSpindle(), geometry, False)
                except Exception as e2:
                    print(f"Error adding rigid body: {e2}")

def CreateFSIBlade(blade, terrain):
    """
    Add blade as FSI solid to the CRM terrain.
    
    Args:
        blade: ChBody blade object
        terrain: CRMTerrain object
    """
    geometry = chrono.ChBodyGeometry()
    
    # Create box geometry for the blade
    box_size = chrono.ChVector3d(1.5, 0.5, 0.05)
    box_pos = chrono.ChVector3d(0, -0.1, 0)
    rot = chrono.QuatFromAngleY(chrono.CH_PI / 2)
    geometry.coll_boxes.append(chrono.BoxShape(box_pos, rot, box_size))
    
    terrain.AddRigidBody(blade, geometry, False)
    print("Added blade to FSI system")

def CreateVehicle(init_pos):
    """
    Create and initialize the Gator vehicle with blade attachment.
    
    Args:
        init_pos: ChCoordsys initial position and orientation
        
    Returns:
        tuple: (gator, blade, motor_yaw, motor_pitch, motor_vertical)
    """
    fea_tires = False
    
    # Create Gator vehicle
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_SMC)
    gator.SetChassisCollisionType(veh.CollisionType_NONE)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(init_pos)
    gator.SetBrakeType(veh.BrakeType_SIMPLE)
    gator.SetTireType(veh.TireModelType_RIGID_MESH)
    gator.SetTireStepSize(1e-3)
    gator.SetAerodynamicDrag(0.5, 5.0, 1.2)
    gator.EnableBrakeLocking(True)
    
    gator.Initialize()
    
    gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
    gator.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
    gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    gator.SetTireVisualizationType(chrono.VisualizationType_MESH)
    
    # Create blade body
    contact_mat = chrono.ChContactMaterialNSC()
    blade = chrono.ChBodyEasyMesh("data/vehicle/gator/gator_frontblade.obj", 1000, True, True, False)
    offsetpos = chrono.ChVector3d(1.75, 0, -0.3)
    
    blade.SetPos(gator.GetChassisBody().TransformPointLocalToParent(offsetpos))
    blade.SetRot(gator.GetChassisBody().GetRot() * chrono.Q_ROTATE_Y_TO_X * chrono.QuatFromAngleX(-chrono.CH_PI_2))
    blade.SetMass(0.1)
    blade.SetFixed(False)
    gator.GetSystem().AddBody(blade)
    
    # Create virtual bodies for motor hierarchy
    vir_yaw_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, False, False, contact_mat)
    vir_yaw_body.SetPos(gator.GetChassisBody().TransformPointLocalToParent(offsetpos))
    gator.GetSystem().AddBody(vir_yaw_body)
    
    vir_pitch_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, False, False, contact_mat)
    vir_pitch_body.SetPos(gator.GetChassisBody().TransformPointLocalToParent(offsetpos))
    gator.GetSystem().AddBody(vir_pitch_body)
    
    vir_vertical_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, False, False, contact_mat)
    
    # Create motors
    motor_yaw = chrono.ChLinkMotorRotationAngle()
    motor_pitch = chrono.ChLinkMotorRotationAngle()
    motor_vertical = chrono.ChLinkMotorLinearPosition()
    
    # Initialize motors
    motor_yaw.Initialize(vir_yaw_body, blade, chrono.ChFramed(vir_yaw_body.GetPos()))
    motor_pitch.Initialize(vir_yaw_body, vir_pitch_body, chrono.ChFramed(vir_pitch_body.GetPos(), chrono.Q_ROTATE_Y_TO_Z))
    motor_vertical.Initialize(gator.GetChassisBody(), vir_pitch_body, chrono.ChFramed(vir_vertical_body.GetPos()))
    
    gator.GetSystem().Add(motor_yaw)
    gator.GetSystem().Add(motor_pitch)
    gator.GetSystem().Add(motor_vertical)
    
    return gator, blade, motor_yaw, motor_pitch, motor_vertical, fea_tires

def LoadBladePvControlData(filename, push_seq):
    """
    Load blade pitch and vertical control data from CSV file.
    
    Args:
        filename: Control file path
        push_seq: Push sequence identifier (for fallback)
        
    Returns:
        list: List of [pitch, vertical] control pairs
    """
    pv_data = []
    
    try:
        if not os.path.exists(filename):
            print(f"Warning: Control file {filename} not found. Using hardcoded fallback values.")
            return _get_fallback_control_data(push_seq)
        
        with open(filename, 'r') as file:
            csv_reader = csv.reader(file)
            valid_lines_read = 0
            
            for line_num, row in enumerate(csv_reader, 1):
                if valid_lines_read >= 2:  # Only read first 2 pairs
                    break
                    
                # Skip empty lines
                if not row or (len(row) == 1 and not row[0].strip()):
                    continue
                
                if len(row) < 2:
                    print(f"Warning: Line {line_num} in {filename} has insufficient data. Expected 2 values, got {len(row)}")
                    continue
                
                try:
                    # Parse pitch and vertical values
                    pitch = float(row[0].strip())
                    vertical = float(row[1].strip())
                    pv_data.append([pitch, vertical])
                    valid_lines_read += 1
                except ValueError as e:
                    print(f"Warning: Invalid number format in line {line_num} of {filename}: {e}")
                    continue
        
        if valid_lines_read < 2:
            print(f"Warning: Control file {filename} contains insufficient valid data. Expected 2 pairs, found {valid_lines_read}. Using fallback values.")
            return _get_fallback_control_data(push_seq)
        
        if valid_lines_read > 2:
            print(f"Warning: Control file {filename} contains more than 2 valid data rows. Using only the first 2.")
            pv_data = pv_data[:2]
            
    except Exception as e:
        print(f"Error reading control file {filename}: {e}")
        print("Using hardcoded fallback values.")
        return _get_fallback_control_data(push_seq)
    
    print(f"Successfully loaded blade P,V control data from {filename} ({len(pv_data)} pairs):")
    for i, data in enumerate(pv_data):
        print(f"  Set {i + 1}: Pitch={data[0]}, Vertical={data[1]}")
    
    return pv_data

def _get_fallback_control_data(push_seq):
    """
    Get hardcoded fallback control data when file loading fails.
    
    Args:
        push_seq: Push sequence identifier
        
    Returns:
        list: List of [pitch, vertical] control pairs
    """
    pv_data = []
    
    if push_seq == "firstpush":
        # First push values from 0.370000_firstpush.txt
        pv_data.append([0.0, -0.0007582007674500346])        # Set 1: Pitch=0.0, Vertical=-0.000758
        pv_data.append([0.5235987901687622, 0.00917821191251278]) # Set 2: Pitch=0.524, Vertical=0.009178
    elif push_seq == "secondpush":
        # Second push values from 0.370000_secondpush.txt
        pv_data.append([0.0, 0.0009243348031304777])        # Set 1: Pitch=0.0, Vertical=0.000924
        pv_data.append([0.0, -0.05000000074505806])         # Set 2: Pitch=0.0, Vertical=-0.050000
    else:
        # Default values if push_seq doesn't match expected patterns
        pv_data.append([0.0, -0.0007582007674500346])        # Default first set
        pv_data.append([0.5235987901687622, 0.00917821191251278]) # Default second set
    
    print(f"Using fallback blade P,V control data ({len(pv_data)} pairs):")
    for i, data in enumerate(pv_data):
        print(f"  Set {i + 1}: Pitch={data[0]}, Vertical={data[1]}")
    
    return pv_data

def GetProblemSpecs():
    """
    Get problem specifications using CLI argument parsing.
    
    Returns:
        tuple: (pile_max_height, push_seq, veh_init_state)
    """
    parser = argparse.ArgumentParser(description='Soil Leveling Validation Configuration')
    
    # Add options for pile_height with its default value
    parser.add_argument('--pile_height', type=float, default=0.37,
                       help='Maximum pile height (default: 0.37)')
    
    # Add option for push sequence
    parser.add_argument('--push_seq', type=str, default='firstpush',
                       choices=['firstpush', 'secondpush'],
                       help='Push sequence type (default: firstpush)')
    
    # Set default values for other parameters
    veh_init_state = [-2.0, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0]
    
    # Parse command-line arguments
    args = parser.parse_args()
    
    pile_max_height = args.pile_height
    push_seq = args.push_seq
    
    # Display help if no arguments provided (similar to C++ version)
    if len(sys.argv) == 1:
        print("Using default parameters. Override with command line options:")
        print("  --pile_height FLOAT    Maximum pile height (default: 0.37)")
        print("  --push_seq STRING      Push sequence type: firstpush or secondpush (default: firstpush)")
        print("  --help                 Show this help message")
        print()
    
    print(f"Using parameters:")
    print(f"pile_max_height: {pile_max_height}")
    print(f"push_seq: {push_seq}")
    
    return pile_max_height, push_seq, veh_init_state

# Set data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ----------------
# Problem settings
# ----------------
print(f"CHRONO_DATA_DIR: {chrono.GetChronoDataPath()}")

target_speed = 1.0
tend = 20
verbose = True

# Visualization settings
render = True
render_fps = 20
control_step_size = 1/20
visualization_sph = True
visualization_bndry_bce = False
visualization_rigid_bce = True
chase_cam = False

# CRM material properties
density = 1700
cohesion = 5e3
friction = 0.8
youngs_modulus = 1e6
poisson_ratio = 0.3

# CRM (moving) active box dimension
active_box_hdim = 2
settling_time = 0

# Set SPH spacing
spacing = 0.02

# Terrain dimensions
terrain_length = 10
terrain_width = 4

# Vehicle initial position
vehicle_init_x = -2.0
vehicle_init_y = 0
vehicle_init_z = 0.3
vehicle_back_x = -2.0

# Process command-line parameters (simplified)
pile_max_height, push_seq, veh_init_state = GetProblemSpecs()

vehicle_state = VehicleState.FORWARD_TO_POSITIVE
t_back = -1.0
t_switch = -1.0

# --------------
# Create vehicle
# --------------

if push_seq == "firstpush":
    init_pos = chrono.ChCoordsysd(chrono.ChVector3d(-2.0, 0.0, 0.3), chrono.QuatFromAngleZ(0.0))
else:  # secondpush
    pos = chrono.ChVector3d(veh_init_state[0], veh_init_state[1], veh_init_state[2])
    rot = chrono.ChQuaterniond(veh_init_state[3], veh_init_state[4], veh_init_state[5], veh_init_state[6])
    init_pos = chrono.ChCoordsysd(pos, rot)

print("Create vehicle...")
gator, blade, motor_yaw, motor_pitch, motor_vertical, fea_tires = CreateVehicle(init_pos)
print("Finished creating vehicle")
sysMBS = gator.GetSystem()

# ---------------------------------
# Set solver and integrator for MBD
# ---------------------------------

if fea_tires:
    step_size = 1e-4
    integrator_type = chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED
    num_threads_chrono = 8
    num_threads_collision = 1
    num_threads_eigen = 1
    num_threads_pardiso = 8
    
    # Set solver and integrator
    sysMBS.SetSolver(mkl.ChSolverPardisoMKL(num_threads_pardiso))
    sysMBS.SetTimestepperType(integrator_type)
    sysMBS.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen)
else:
    step_size = 5e-4
    solver_type = chrono.ChSolver.Type_BARZILAIBORWEIN
    integrator_type = chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED
    num_threads_chrono = 8
    num_threads_collision = 1
    num_threads_eigen = 1
    
    # Set solver and integrator
    sysMBS.SetSolverType(solver_type)
    sysMBS.SetTimestepperType(integrator_type)
    sysMBS.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen)

# Set collision system
sysMBS.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ----------------------
# Create the CRM terrain
# ----------------------

terrain = veh.CRMTerrain(sysMBS, spacing)
sysFSI = terrain.GetSystemFSI()
terrain.SetVerbose(verbose)
terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
terrain.SetStepSizeCFD(step_size)
# terrain.GetFluidSystemSPH().EnableCudaErrorCheck(False)  # Comment: May not be available in Python bindings

# Set SPH parameters and soil material properties
mat_props = fsi.ElasticMaterialProperties()
mat_props.density = density
mat_props.Young_modulus = youngs_modulus
mat_props.Poisson_ratio = poisson_ratio
mat_props.mu_I0 = 0.04
mat_props.mu_fric_s = friction
mat_props.mu_fric_2 = friction
mat_props.average_diam = 0.005
mat_props.cohesion_coeff = cohesion
terrain.SetElasticSPH(mat_props)

# Set SPH solver parameters
sph_params = fsi.SPHParameters()
sph_params.initial_spacing = spacing
sph_params.d0_multiplier = 1
sph_params.kernel_threshold = 0.8
sph_params.artificial_viscosity = 0.5
sph_params.consistent_gradient_discretization = False
sph_params.consistent_laplacian_discretization = False
terrain.SetSPHParameters(sph_params)

# Set output level from SPH simulation
# Comment: terrain.SetOutputLevel() may not be available in Python bindings

# Add vehicle wheels as FSI solids
CreateFSIWheels(gator, terrain)

# Add blade as FSI solid
CreateFSIBlade(blade, terrain)

terrain.SetActiveDomain(chrono.ChVector3d(active_box_hdim, active_box_hdim, active_box_hdim))
terrain.SetActiveDomainDelay(settling_time)

# Construct the terrain and associated path
print("Create terrain...")
# Using HEIGHT_MAP patch type
terrain.Construct("data/vehicle/terrain/terrain.bmp",
                  terrain_length, terrain_width,
                  chrono.ChVector2d(0, pile_max_height),
                  0.45,
                  True,
                  chrono.ChVector3d(0, 0, 0),
                  fsi.BoxSide_Z_NEG)

# Initialize the terrain system
terrain.Initialize()

aabb = terrain.GetSPHBoundingBox()
print(f"  SPH particles:     {terrain.GetNumSPHParticles()}")
print(f"  Bndry BCE markers: {terrain.GetNumBoundaryBCEMarkers()}")
print(f"  SPH AABB:          {aabb.min}   {aabb.max}")

# Set maximum vehicle X location (based on CRM patch size)
x_max = aabb.max.x - 4.5

# --------------------------------
# Create the driver
# --------------------------------
print("Create driver...")
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# -----------------------------
# Create run-time visualization
# -----------------------------
vis = None
if render:
    # Comment: VSG visualization setup - may need adaptation for Python bindings
    try:
        # FSI plugin
        col_callback = fsi.ParticleHeightColorCallback(aabb.min.z, aabb.max.z)
        visFSI = fsi.ChFsiVisualizationVSG(sysFSI)
        visFSI.EnableFluidMarkers(visualization_sph)
        visFSI.EnableBoundaryMarkers(visualization_bndry_bce)
        visFSI.EnableRigidBodyMarkers(visualization_rigid_bce)
        visFSI.SetSPHColorCallback(col_callback, chrono.ChColormap.Type_BROWN)
        
        # Wheeled vehicle VSG visual system
        visVSG = veh.ChWheeledVehicleVisualSystemVSG()
        visVSG.AttachVehicle(gator.GetVehicle())
        visVSG.AttachPlugin(visFSI)
        visVSG.SetWindowTitle("Wheeled vehicle on CRM deformable terrain")
        visVSG.SetWindowSize(1280, 800)
        visVSG.SetWindowPosition(100, 100)
        visVSG.EnableSkyBox()
        visVSG.SetLightIntensity(1.0)
        visVSG.SetLightDirection(1.5 * chrono.CH_PI_2, chrono.CH_PI_4)
        visVSG.SetCameraAngleDeg(40)
        visVSG.SetChaseCamera(chrono.VNULL, 6.0, 2.0)
        visVSG.SetChaseCameraPosition(chrono.ChVector3d(0, 8, 1.5))
        
        visVSG.Initialize()
        vis = visVSG
    except Exception as e:
        print(f"Visualization setup failed: {e}")
        render = False

# ---------------
# Simulation loop
# ---------------

time = 0
sim_frame = 0
render_frame = 0
saveframes = True
print("Start simulation...")

# Create output directory
out_dir = f"../data/output/{pile_max_height}/soil_leveling_{push_seq}"
os.makedirs(out_dir, exist_ok=True)

# Create particle frames directory
pc_dir = f"../data/output/{pile_max_height}/soil_leveling_{push_seq}/pc_frames"
os.makedirs(pc_dir, exist_ok=True)

simulation_complete = False
saved_particle = False

# Load blade control data
blade_pv_filename = f"../data/control_commands/{pile_max_height}_{push_seq}.txt"
blade_pv_setpoints = LoadBladePvControlData(blade_pv_filename, push_seq)

while time < tend and not simulation_complete:
    veh_loc = gator.GetVehicle().GetPos()
    veh_speed = gator.GetVehicle().GetSpeed()
    veh_rot = gator.GetVehicle().GetRot()
    front_load = blade.GetAppliedForce()
    blade_loc = gator.GetChassisBody().TransformPointParentToLocal(blade.GetPos())
    blade_rot = gator.GetChassisBody().GetRot().GetInverse() * blade.GetRot()
    blade_rot_angles = blade_rot.GetCardanAnglesZYX()
    engine_rpm = gator.GetVehicle().GetEngine().GetMotorSpeed()
    engine_torque = gator.GetVehicle().GetEngine().GetOutputMotorshaftTorque()
    
    driver_inputs = veh.DriverInputs()
    
    # State machine to control vehicle movement
    if vehicle_state == VehicleState.FORWARD_TO_POSITIVE:
        driver_inputs.m_throttle = 1.0
        driver_inputs.m_braking = 0.0
        gator.GetVehicle().GetTransmission().SetGear(1)
        
        if time > 1.0 and time < 3.5:
            motor_pitch.SetAngleFunction(chrono.ChFunctionConst(blade_pv_setpoints[0][0]))
            motor_vertical.SetMotionFunction(chrono.ChFunctionConst(blade_pv_setpoints[0][1]))
        elif time > 3.5 and time < 6.0:
            motor_pitch.SetAngleFunction(chrono.ChFunctionConst(blade_pv_setpoints[1][0]))
            motor_vertical.SetMotionFunction(chrono.ChFunctionConst(blade_pv_setpoints[1][1]))
        
        if time >= 6.0 or veh_loc.x >= 2.5:
            if verbose:
                print("Transition: FORWARD_TO_POSITIVE -> WAIT_AT_POSITIVE")
            vehicle_state = VehicleState.WAIT_AT_POSITIVE
            t_switch = time
    
    elif vehicle_state == VehicleState.WAIT_AT_POSITIVE:
        driver_inputs.m_throttle = 0.0
        driver_inputs.m_braking = 1.0
        motor_pitch.SetAngleFunction(chrono.ChFunctionConst(0.0))
        motor_vertical.SetMotionFunction(chrono.ChFunctionConst(-0.1))
        
        if time >= t_switch + 1.0:
            if verbose:
                print("Transition: WAIT_AT_POSITIVE -> BACKWARD_TO_NEGATIVE")
            vehicle_state = VehicleState.BACKWARD_TO_NEGATIVE
            gator.GetVehicle().GetTransmission().SetGear(-1)
    
    elif vehicle_state == VehicleState.BACKWARD_TO_NEGATIVE:
        driver_inputs.m_throttle = 0.1
        driver_inputs.m_braking = 0.0
        
        dist_to_target = chrono.ChVector3d(veh_loc.x - vehicle_back_x, 0, 0).Length()
        if dist_to_target < 0.5:
            t_back = time
            if verbose:
                print(f"Reached backward target at X={vehicle_back_x} at time t_back = {t_back}s")
                print("Transition: BACKWARD_TO_NEGATIVE -> WAIT_AT_NEGATIVE")
            vehicle_state = VehicleState.WAIT_AT_NEGATIVE
            # Save SPH data and end simulation
            if not saved_particle:
                try:
                    sysFSI.GetFluidSystemSPH().SaveParticleData(pc_dir)
                    print(f"Particle data saved to {out_dir}")
                    saved_particle = True
                except Exception as e:
                    print(f"Warning: Could not save particle data: {e}")

    
    elif vehicle_state == VehicleState.WAIT_AT_NEGATIVE:
        if t_back < 0:
            driver_inputs.m_throttle = 0.0
            driver_inputs.m_braking = 1.0
        else:
            driver_inputs.m_throttle = 0.0
            driver_inputs.m_braking = 1.0
            motor_vertical.SetMotionFunction(chrono.ChFunctionConst(0.0))
            if time >= t_back + 1.0:
                vehicle_state = VehicleState.FORWARD_TO_POSITIVE_AGAIN
                gator.GetVehicle().GetTransmission().SetGear(1)
                # Read the new control command file
                new_push_seq = "secondpush"
                new_control_command_file = f"../data/control_commands/{pile_max_height}_{new_push_seq}.txt"
                blade_pv_setpoints = LoadBladePvControlData(new_control_command_file, new_push_seq)
                print(f"Loaded new control command file: {new_control_command_file}")
    
    elif vehicle_state == VehicleState.FORWARD_TO_POSITIVE_AGAIN:
        driver_inputs.m_throttle = 1.0
        driver_inputs.m_braking = 0.0
        if time > t_back + 2.0 and time < t_back + 4.5:
            motor_pitch.SetAngleFunction(chrono.ChFunctionConst(blade_pv_setpoints[0][0]))
            motor_vertical.SetMotionFunction(chrono.ChFunctionConst(blade_pv_setpoints[0][1]))
        elif time > t_back + 4.5 and time < t_back + 7.0:
            motor_pitch.SetAngleFunction(chrono.ChFunctionConst(blade_pv_setpoints[1][0]))
            motor_vertical.SetMotionFunction(chrono.ChFunctionConst(blade_pv_setpoints[1][1]))
    
    # Run-time visualization
    if render and vis and time >= render_frame / render_fps:
        if not vis.Run():
            break
        vis.Render()
        render_frame += 1
    
    if not render:
        print(f"{time}  {terrain.GetRtfCFD()}  {terrain.GetRtfMBD()}")
    
    # Synchronize systems
    driver.Synchronize(time)
    if vis:
        vis.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    
    # Advance system state
    driver.Advance(step_size)
    if vis:
        vis.Advance(step_size)
    terrain.Advance(step_size)
    
    time += step_size
    sim_frame += 1

print("Simulation completed")
