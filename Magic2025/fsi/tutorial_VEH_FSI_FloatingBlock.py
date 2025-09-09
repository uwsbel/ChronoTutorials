# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Author: Luning Bakke, Huzaifa Unjhawala
# =============================================================================
#
# Demo showing a Polaris vehicle moving over a floating block
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.fsi as fsi
import pychrono.vsg as vsg
import os
import time

def CreateVehicle(sys, sysSPH, sysFSI, init_pos):
    """
    Create and initialize the Polaris vehicle.
    
    Args:
        sys: ChSystemSMC object
        sysSPH: ChFsiFluidSystemSPH object
        sysFSI: ChFsiSystemSPH object
        init_pos: Initial position and orientation
    
    Returns:
        WheeledVehicle object
    """
    vehicle_json = "Polaris/Polaris.json"
    engine_json = "Polaris/Polaris_EngineSimpleMap.json"
    transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json"
    tire_json = "Polaris/Polaris_RigidTire.json"

    # Create and initialize the vehicle
    vehicle = veh.WheeledVehicle(sys, veh.GetDataFile(vehicle_json))
    vehicle.Initialize(init_pos)
    vehicle.GetChassis().SetFixed(False)
    vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)

    # Create and initialize the powertrain system
    engine = veh.ReadEngineJSON(veh.GetDataFile(engine_json))
    transmission = veh.ReadTransmissionJSON(veh.GetDataFile(transmission_json))
    powertrain = veh.ChPowertrainAssembly(engine, transmission)
    vehicle.InitializePowertrain(powertrain)

    # Create and initialize the tires
    for axle in vehicle.GetAxles():
        for wheel in axle.GetWheels():
            tire = veh.ReadTireJSON(veh.GetDataFile(tire_json))
            vehicle.InitializeTire(tire, wheel, chrono.VisualizationType_MESH)

    # Create contact material for wheels
    cmaterial = chrono.ChContactMaterialSMC()
    cmaterial.SetYoungModulus(1e8)
    cmaterial.SetFriction(0.9)
    cmaterial.SetRestitution(0.4)
    
    mesh_filename = veh.GetDataFile("Polaris/meshes/Polaris_tire_collision.obj")
    
    # Create geometry for rigid wheels
    geometry = chrono.ChBodyGeometry()
    geometry.coll_meshes.append(chrono.TrimeshShape(chrono.VNULL, chrono.QUNIT, mesh_filename, chrono.VNULL))

    trimesh = chrono.ChTriangleMeshConnected()
    scale_ratio = 1.0
    trimesh = trimesh.CreateFromWavefrontFile(mesh_filename, False, True)
    trimesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(scale_ratio))
    trimesh.RepairDuplicateVertexes(1e-9)
    wheel_shape = chrono.ChCollisionShapeTriangleMesh(cmaterial, trimesh, False, False, 0.005)

    # Create wheel BCE markers
    for axle in vehicle.GetAxles():
        for wheel in axle.GetWheels():
            wheel.GetSpindle().AddCollisionShape(wheel_shape)
            wheel.GetSpindle().EnableCollision(True)
            tire = wheel.GetTire()
            sysFSI.AddFsiBody(wheel.GetSpindle(), geometry, False)


    return vehicle

def main():
    # Simulation parameters
    t_end = 3
    verbose = True
    output = False
    output_fps = 20
    render = True
    render_fps = 100
    snapshots = False
    ps_freq = 1

    # Dimension of the fluid domain
    fxDim = 4
    fyDim = 2.0
    fzDim = 1.0

    # Dimension of the space domain
    bxDim = fxDim
    byDim = fyDim
    bzDim = fzDim + 0.05

    # SPH parameters
    initial_spacing = 0.1
    step_size = 1e-4

    # Create a physics system and an FSI system
    sysMBS = chrono.ChSystemSMC()
    sysSPH = fsi.ChFsiFluidSystemSPH()
    sysFSI = fsi.ChFsiSystemSPH(sysMBS, sysSPH)
    sysMBS.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sysMBS.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    # Set SPH parameters directly instead of using JSON file
    sysFSI.SetStepSizeCFD(step_size)
    sysFSI.SetStepsizeMBD(step_size)
    
    # Set gravitational acceleration
    sysFSI.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    fluid_props = fsi.FluidProperties()
    fluid_props.density = 1000
    fluid_props.viscosity = 5.0
    sysSPH.SetCfdSPH(fluid_props)

    sph_params = fsi.SPHParameters()
    sph_params.integration_scheme = fsi.IntegrationScheme_RK2
    sph_params.initial_spacing = initial_spacing
    sph_params.d0_multiplier = 1.2
    sph_params.max_velocity = 10.0
    sph_params.shifting_method = fsi.ShiftingMethod_PPST
    sph_params.shifting_ppst_push = 3.0
    sph_params.shifting_ppst_pull = 1.0
    sph_params.artificial_viscosity = 0.03
    sph_params.boundary_method = fsi.BoundaryMethod_ADAMI
    sph_params.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_UNILATERAL
    sph_params.use_delta_sph = True
    sph_params.delta_sph_coefficient = 0.1
    sph_params.num_proximity_search_steps = ps_freq
    sph_params.eos_type = fsi.EosType_TAIT
    sysSPH.SetSPHParameters(sph_params)

    # Set up the periodic boundary condition (only in Y direction)
    cMin = chrono.ChVector3d(-bxDim / 2 - 5 * initial_spacing, -byDim / 2 - initial_spacing / 2, -5 * initial_spacing)
    cMax = chrono.ChVector3d(+bxDim / 2 + 5 * initial_spacing, +byDim / 2 + initial_spacing / 2, bzDim + 5 * initial_spacing)
    sysSPH.SetComputationalDomain(chrono.ChAABB(cMin, cMax), fsi.BC_Y_PERIODIC)

    # Create Fluid region and discretize with SPH particles
    boxCenter = chrono.ChVector3d(-bxDim / 2 + fxDim / 2, 0.0, fzDim / 2)
    boxHalfDim = chrono.ChVector3d(fxDim / 2 - initial_spacing, fyDim / 2, fzDim / 2 - initial_spacing)

    # Use a Chrono sampler to create a bucket of points
    sampler = chrono.ChGridSamplerd(initial_spacing)
    points = sampler.SampleBox(boxCenter, boxHalfDim)

    # Add fluid particles from the sampler points to the FSI system
    numPart = len(points)
    gz = abs(sysSPH.GetGravitationalAcceleration().z)
    for i in range(numPart):
        # Calculate the pressure of a steady state (p = rho*g*h)
        pre_ini = sysSPH.GetDensity() * gz * (-points[i].z + fzDim)
        rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed())
        sysSPH.AddSPHParticle(points[i], rho_ini, pre_ini, sysSPH.GetViscosity())

    # Create contact material for collision shapes
    cmaterial = chrono.ChContactMaterialSMC()
    cmaterial.SetYoungModulus(1e8)
    cmaterial.SetFriction(0.9)
    cmaterial.SetRestitution(0.4)

    # Create container collision and visualization shapes
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(True)

    # Bottom wall
    dim = chrono.ChVector3d(bxDim + 4 * initial_spacing, byDim + 0 * initial_spacing, 2 * initial_spacing)
    loc = chrono.ChVector3d(0, 0, -initial_spacing)
    vis_shape = chrono.ChVisualShapeBox(dim)
    vis_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground.AddVisualShape(vis_shape, chrono.ChFramed(loc, chrono.QUNIT))

    # Left wall of the container
    dim = chrono.ChVector3d(2 * initial_spacing, byDim, bzDim + 4 * initial_spacing)
    loc = chrono.ChVector3d(+bxDim / 2 + initial_spacing, 0, bzDim / 2)
    vis_shape = chrono.ChVisualShapeBox(dim)
    vis_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground.AddVisualShape(vis_shape, chrono.ChFramed(loc, chrono.QUNIT))
    ground.AddCollisionShape(chrono.ChCollisionShapeBox(cmaterial, dim), chrono.ChFramed(loc, chrono.QUNIT))

    # Right wall of the container
    dim = chrono.ChVector3d(2 * initial_spacing, byDim, bzDim + 4 * initial_spacing)
    loc = chrono.ChVector3d(-bxDim / 2 - initial_spacing, 0, bzDim / 2)
    vis_shape = chrono.ChVisualShapeBox(dim)
    vis_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground.AddVisualShape(vis_shape, chrono.ChFramed(loc, chrono.QUNIT))
    ground.AddCollisionShape(chrono.ChCollisionShapeBox(cmaterial, dim), chrono.ChFramed(loc, chrono.QUNIT))

    # Left platform
    dim = chrono.ChVector3d(bxDim, byDim, 2 * initial_spacing)
    loc = chrono.ChVector3d(-bxDim / 2 - bxDim / 2 - 3 * initial_spacing, 0, bzDim + initial_spacing)
    vis_shape = chrono.ChVisualShapeBox(dim)
    vis_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground.AddVisualShape(vis_shape, chrono.ChFramed(loc, chrono.QUNIT))
    ground.AddCollisionShape(chrono.ChCollisionShapeBox(cmaterial, dim), chrono.ChFramed(loc, chrono.QUNIT))

    # Right platform
    dim = chrono.ChVector3d(bxDim, byDim, 2 * initial_spacing)
    loc = chrono.ChVector3d(+bxDim / 2 + bxDim / 2 + 3 * initial_spacing, 0, bzDim + initial_spacing)
    vis_shape = chrono.ChVisualShapeBox(dim)
    vis_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground.AddVisualShape(vis_shape, chrono.ChFramed(loc, chrono.QUNIT))
    ground.AddCollisionShape(chrono.ChCollisionShapeBox(cmaterial, dim), chrono.ChFramed(loc, chrono.QUNIT))

    sysMBS.AddBody(ground)

    # Add FSI container
    ground_bce = sysSPH.CreatePointsBoxContainer(chrono.ChVector3d(bxDim, byDim, bzDim), chrono.ChVector3i(2, 0, -1))
    sysFSI.AddFsiBody(ground, ground_bce, chrono.ChFramed(chrono.ChVector3d(0, 0, bzDim / 2), chrono.QUNIT), False)

    # Floating block size and density
    plate_size = chrono.ChVector3d(0.9 * fxDim, 0.7 * fyDim, 4 * initial_spacing)
    plate_density = 400
    plate_volume = plate_size.x * plate_size.y * plate_size.z
    plate_mass = plate_volume * plate_density
    plate_inertia = chrono.ChVector3d(
        (1.0 / 12.0) * plate_mass * (plate_size.y * plate_size.y + plate_size.z * plate_size.z),  # I_xx
        (1.0 / 12.0) * plate_mass * (plate_size.x * plate_size.x + plate_size.z * plate_size.z),  # I_yy
        (1.0 / 12.0) * plate_mass * (plate_size.x * plate_size.x + plate_size.y * plate_size.y)   # I_zz
    )

    plate_center = chrono.ChVector3d(0, 0, fzDim + plate_size.z * 0.5)

    floating_plate = chrono.ChBody()
    floating_plate.SetPos(plate_center)
    floating_plate.SetFixed(False)
    floating_plate.SetMass(plate_mass)
    floating_plate.SetInertiaXX(plate_inertia)
    floating_plate.EnableCollision(True)

    vis_shape = chrono.ChVisualShapeBox(plate_size)
    vis_shape.SetTexture(chrono.GetChronoDataFile("textures/spheretexture.png"), 4, 2)
    floating_plate.AddVisualShape(vis_shape, chrono.ChFramed())
    floating_plate.AddCollisionShape(chrono.ChCollisionShapeBox(cmaterial, plate_size), chrono.ChFramed())

    sysMBS.AddBody(floating_plate)

    bce = sysSPH.CreatePointsBoxInterior(plate_size)
    sysFSI.AddFsiBody(floating_plate, bce, chrono.ChFramed(), False)

    # Create vehicle
    veh_init_pos = chrono.ChVector3d(-bxDim / 2 - bxDim * chrono.CH_1_3, 0, bzDim + 3 * initial_spacing + 0.1)
    vehicle = CreateVehicle(sysMBS, sysSPH, sysFSI, chrono.ChCoordsysd(veh_init_pos, chrono.QUNIT))

    # Complete construction of the FSI system
    sysFSI.Initialize()

    # Output directories
    out_dir = chrono.GetChronoOutputPath() + "VEH_FSI_FloatingBlock_" + str(ps_freq)

    try:
        os.makedirs(out_dir, exist_ok=True)
    except OSError as e:
        print(f"Error creating directory {out_dir}: {e}")
        return 1
    
    out_dir = os.path.join(out_dir, sysSPH.GetSphIntegrationSchemeString() + "_ps" + str(ps_freq))

    try:
        os.makedirs(out_dir, exist_ok=True)
    except OSError as e:
        print(f"Error creating directory {out_dir}: {e}")
        return 1

    if output:
        try:
            os.makedirs(os.path.join(out_dir, "particles"), exist_ok=True)
            os.makedirs(os.path.join(out_dir, "fsi"), exist_ok=True)
        except OSError as e:
            print(f"Error creating output directories: {e}")
            return 1
    
    if snapshots:
        try:
            os.makedirs(os.path.join(out_dir, "snapshots"), exist_ok=True)
        except OSError as e:
            print(f"Error creating snapshots directory: {e}")
            return 1

    # Create a run-time visualizer
    vis = None

    if render:
        # FSI plugin
        col_callback = fsi.ParticleVelocityColorCallback(0, 5.0)

        visFSI = fsi.ChFsiVisualizationVSG(sysFSI)
        visFSI.EnableFluidMarkers(True)
        visFSI.EnableBoundaryMarkers(False)
        visFSI.EnableRigidBodyMarkers(False)
        visFSI.EnableFlexBodyMarkers(False)
        visFSI.SetSPHColorCallback(col_callback)

        # Vehicle VSG visual system (attach visFSI as plugin)
        visVSG = veh.ChWheeledVehicleVisualSystemVSG()
        visVSG.AttachVehicle(vehicle)
        visVSG.AttachPlugin(visFSI)
        visVSG.SetWindowTitle("Floating Block")
        visVSG.SetWindowSize(1280, 800)
        visVSG.SetWindowPosition(100, 100)
        visVSG.SetLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        visVSG.SetBackgroundColor(chrono.ChColor(0.1, 0.15, 0.2))
        visVSG.SetChaseCameraPosition(chrono.ChVector3d(0, -7 * byDim, 3 + bzDim / 2), chrono.ChVector3d(0, 0, bzDim / 2))
        visVSG.SetLightIntensity(0.9)
        visVSG.SetLightDirection(-chrono.CH_PI_2, chrono.CH_PI_4)

        visVSG.Initialize()
        vis = visVSG

    # Start the simulation
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0
    driver_inputs.m_braking = 0
    driver_inputs.m_steering = 0
    
    dT = sysFSI.GetStepSizeCFD()
    time_val = 0
    sim_frame = 0
    out_frame = 0
    render_frame = 0

    x_max = bxDim / 2 + bxDim / 3
    timer = chrono.ChTimer()
    timer.start()
    
    while time_val < t_end:
        # Save data of the simulation
        if output and time_val >= out_frame / output_fps:
            print("------- OUTPUT")
            sysSPH.SaveParticleData(os.path.join(out_dir, "particles"))
            sysSPH.SaveSolidData(os.path.join(out_dir, "fsi"), time_val)
            out_frame += 1

        # Simple Vehicle Control
        if time_val < 0.4:
            driver_inputs.m_throttle = 0
            driver_inputs.m_braking = 1
        else:
            driver_inputs.m_throttle = min(0.5 * time_val, 1.0)
            driver_inputs.m_braking = 0

        # Stop the vehicle when it reaches the end of the domain
        if vehicle.GetPos().x > x_max:
            driver_inputs.m_throttle = 0
            driver_inputs.m_braking = 1

        # Render FSI system
        if render and time_val >= render_frame / render_fps:
            if not vis.Run():
                break
            vis.Render()

            if snapshots:
                if verbose:
                    print(f" -- Snapshot frame {render_frame} at t = {time_val}")
                filename = os.path.join(out_dir, "snapshots", f"img_{render_frame + 1:05d}.bmp")
                vis.WriteImageToFile(filename)

            render_frame += 1

        # Synchronize systems
        vehicle.Synchronize(time_val, driver_inputs)
        if vis:
            vis.Synchronize(time_val, driver_inputs)

        # Advance system state
        sysFSI.DoStepDynamics(dT)
        if vis:
            vis.Advance(dT)

        time_val += dT
        sim_frame += 1
    
    timer.stop()
    print(f"End Time: {t_end}")
    print(f"\nSimulation time: {timer.GetTimeSeconds()} seconds\n")

    return 0

if __name__ == "__main__":
    # Set data path for vehicle files
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")
    
    main()
