# Copyright (c) 2021, SBEL GPU Development Team
# Copyright (c) 2021, University of Wisconsin - Madison
#
# SPDX-License-Identifier: BSD-3-Clause

# =============================================================================
# This demo features a mesh-represented bladed mixer interacting with clump-represented
# DEM particles.
# =============================================================================

import random
import DEME

from DEME import GridSampler


import numpy as np
import os
import time

def force_model():
    model = """
        if (overlapDepth > 0.0) {
            // Material properties

            // DEM force calculation strategies for grain breakage
            float E_cnt, G_cnt, CoR_cnt, mu_cnt, Crr_cnt, E_A, E_B;
            {
                // E and nu are associated with each material, so obtain them this way
                E_A = E[bodyAMatType];
                float nu_A = nu[bodyAMatType];
                E_B = E[bodyBMatType];
                float nu_B = nu[bodyBMatType];
                matProxy2ContactParam<float>(E_cnt, G_cnt, E_A, nu_A, E_B, nu_B);
                // CoR, mu and Crr are pair-wise, so obtain them this way
                CoR_cnt = CoR[bodyAMatType][bodyBMatType];
                mu_cnt = mu[bodyAMatType][bodyBMatType];
                Crr_cnt = Crr[bodyAMatType][bodyBMatType];
            }
            float3 rotVelCPA, rotVelCPB;
            {
                // We also need the relative velocity between A and B in global frame to use in the damping terms
                // To get that, we need contact points' rotational velocity in GLOBAL frame
                // This is local rotational velocity (the portion of linear vel contributed by rotation)
                rotVelCPA = cross(ARotVel, locCPA);
                rotVelCPB = cross(BRotVel, locCPB);
                // This is mapping from local rotational velocity to global
                applyOriQToVector3<float, deme::oriQ_t>(rotVelCPA.x, rotVelCPA.y, rotVelCPA.z, AOriQ.w, AOriQ.x, AOriQ.y,
                                                        AOriQ.z);
                applyOriQToVector3<float, deme::oriQ_t>(rotVelCPB.x, rotVelCPB.y, rotVelCPB.z, BOriQ.w, BOriQ.x, BOriQ.y,
                                                        BOriQ.z);
            }
            float mass_eff, sqrt_Rd, beta;
            float3 vrel_tan;
            float3 delta_tan = make_float3(delta_tan_x, 0.0, delta_tan_z);

            // The (total) relative linear velocity of A relative to B
            const float3 velB2A = (ALinVel + rotVelCPA) - (BLinVel + rotVelCPB);
            const float projection = dot(velB2A, B2A);
            vrel_tan = velB2A - projection * B2A;
            vrel_tan.y = 0.0;

            const float3 v_rot = rotVelCPB - rotVelCPA;
            // This v_rot is only used for identifying resistance direction
            const float v_rot_mag = length(v_rot);
            mass_eff = (AOwnerMass * BOwnerMass) / (AOwnerMass + BOwnerMass);

            // Now we already have sufficient info to update contact history
            {
                delta_tan += ts * vrel_tan;
                const float disp_proj = dot(delta_tan, B2A);
                delta_tan -= disp_proj * B2A;
                delta_time += ts;
            }

            // Normal force part
            sqrt_Rd = sqrt(overlapDepth * (ARadius * BRadius) / (ARadius + BRadius));
            const float Sn = 2. * E_cnt * sqrt_Rd;

            const float loge = (CoR_cnt < DEME_TINY_FLOAT) ? log(DEME_TINY_FLOAT) : log(CoR_cnt);
            beta = loge / sqrt(loge * loge + deme::PI_SQUARED);

            const float k_n = deme::TWO_OVER_THREE * Sn;
            const float gamma_n = deme::TWO_TIMES_SQRT_FIVE_OVER_SIX * beta * sqrt(Sn * mass_eff);

            force += (k_n * overlapDepth + gamma_n * projection) * B2A;

            // Tangential force part
            if (mu_cnt > 0.0) {
                const float kt = 8. * G_cnt * sqrt_Rd;
                const float gt = -deme::TWO_TIMES_SQRT_FIVE_OVER_SIX * beta * sqrt(mass_eff * kt);
                float3 tangent_force = -kt * delta_tan - gt * vrel_tan;
                const float ft = length(tangent_force);
                if (ft > DEME_TINY_FLOAT) {
                    // Reverse-engineer to get tangential displacement
                    const float ft_max = length(force) * mu_cnt;
                    if (ft > ft_max) {
                        tangent_force = (ft_max / ft) * tangent_force;
                        delta_tan = (tangent_force + gt * vrel_tan) / (-kt);
                    }
                } else {
                    tangent_force = make_float3(0, 0, 0);
                }
                // Use force to collect tangent_force
                force += tangent_force;
            }

            // Make sure we update those wildcards (in this case, contact history)
            delta_tan_x = delta_tan.x;
            delta_tan_y = 0.0;
            delta_tan_z = delta_tan.z;
            force.y = 0.0;

            int T_update_freqency = 10000;
            int curr_step = (int)(time / ts);

            if (curr_step % T_update_freqency == 0) {
            
                double force_mag =  k_n * overlapDepth + gamma_n * projection;

                if (force_mag < 0) {
                    force_mag = 0;
                }
                

                // effective contact radius
                double radius_eff = (ARadius * BRadius) / (ARadius + BRadius);
                double radius_contact = powf( 2.f * radius_eff / E_cnt * force_mag, 1.0/3.0);

                // Initialize temperatures and heat flux
                double Q_ij = 0;

                double T_i = Temp_A[AGeo];
                double T_j = Temp_B[BGeo];
                double ks = 20;
                Q_ij = 2. * ks * radius_contact * (T_j - T_i);

                atomicAdd(Q_A + AGeo,  Q_ij);
                atomicAdd(Q_B + BGeo, -Q_ij);

//                if (AGeo > 2000 || BGeo > 2000) {
//                    printf("T_j: %f, T_i: %f, Q_ij: %e, radius_contact: %f, force_mag: %f, AGeo: %d, BGeo: %d\\n", T_j, T_i, Q_ij, radius_contact, force_mag, AGeo, BGeo);
//                }
            }
        }
    """

    return model

if __name__ == "__main__":
    out_dir = "HeatTransfer/"
    out_dir = os.path.join(os.getcwd(), out_dir)
    os.makedirs(out_dir, exist_ok=True)

    drum_temperature = 300
    initial_particle_temperature = 200

    DEMSim = DEME.DEMSolver()

    DEMSim.SetVerbosity("STEP_METRIC")
    DEMSim.SetOutputFormat("CSV")
    DEMSim.SetOutputContent(["ABSV", "XYZ", "GEO_WILDCARD"])
    DEMSim.SetMeshOutputFormat("VTK")

    # If you don't need individual force information, then this option makes the solver run a bit faster.
    DEMSim.SetNoForceRecord(True)

    model2D = DEMSim.DefineContactForceModel(force_model())
    model2D.SetMustHaveMatProp(set(["E", "nu", "CoR", "mu", "Crr"]))
    model2D.SetMustPairwiseMatProp(set(["CoR", "mu", "Crr"]))
    model2D.SetPerContactWildcards(
        set(["delta_time", "delta_tan_x", "delta_tan_y", "delta_tan_z"])
    )
    model2D.SetPerGeometryWildcards(
        set(["Temp", "Q"])
    )


    # E, nu, CoR, mu, Crr... Material properties
    mat_type_mixer = DEMSim.LoadMaterial(
        {"E": 5e6, "nu":  0.3, "CoR":  0.3, "mu":  0.5, "Crr": 0.0})
    mat_type_granular = DEMSim.LoadMaterial(
        {"E":  5e6, "nu":  0.3, "CoR":  0.3, "mu":  0.6, "Crr":  0.0})
    # If you don't have this line, then mu between mixer material and granular material will be 0.35 (average of the two).
    DEMSim.SetMaterialPropertyPair(
        "CoR", mat_type_mixer, mat_type_granular, 0.3)

    step_size = 5e-6
    world_size = 1
    chamber_height = world_size / 3.
    fill_height = chamber_height
    chamber_bottom = -0.4
    fill_bottom = chamber_bottom + chamber_height

    DEMSim.InstructBoxDomainDimension(world_size, world_size, world_size)
    DEMSim.InstructBoxDomainBoundingBC("all", mat_type_granular)

    granular_rad = 0.005
    templates_terrain = []
    num_templates = 2
    for i in range(num_templates):
        templates_terrain.append(
            DEMSim.LoadSphereType(
                granular_rad * granular_rad * granular_rad * 1.5e3 * 4 / 3 * np.pi,
                granular_rad,
                mat_type_granular,
            )
        )
        granular_rad += 0.001 / 2.

    # Sampler uses hex close-packing
    sampler = GridSampler(3.0 * granular_rad)
    fill_center = [0, 0, 0]
    fill_radius = 0.45 - 2. * granular_rad
    input_xyz = sampler.SampleCylinderY(
        fill_center, fill_radius, 0.0)

    template_granular = []
    # Randomly select from pre - generated clump templates
    for i in range(len(input_xyz)):
        template_granular.append(templates_terrain[random.randint(0, num_templates - 1)])



    particles = DEMSim.AddClumps(template_granular, input_xyz)
    num_particles = particles.GetNumSpheres()
    print(f"Total num of particles: {num_particles}")

    particles.AddGeometryWildcard(
        "Temp", [initial_particle_temperature] * num_particles)
    particles.AddGeometryWildcard(
        "Q", [0] * num_particles)


    # Track the mixer
    meshfile_name = "cylinder.obj"
    mixer = DEMSim.AddWavefrontMeshObject(meshfile_name, mat_type_mixer)
    print(f"Total num of triangles: {mixer.GetNumTriangles()}")
    mixer.Move([0, -0.5, 0], [0, 0, 0, 1])
    mixer.SetFamily(10)
    numTriangles = mixer.GetNumTriangles()
    # Define the prescribed motion of mixer
    DEMSim.SetFamilyPrescribedAngVel(10, "0", "1.14159", "0")
    # DEMSim.SetFamilyPrescribedAngVel(10, "0", "0", "0")

    mixer_tracker = DEMSim.Track(mixer)


    DEMSim.SetInitTimeStep(step_size)
    DEMSim.SetGravitationalAcceleration([0, 0, -9.81])
    DEMSim.SetCDUpdateFreq(40)
    # Mixer has a big angular velocity-contributed linear speed at its blades, this is something the solver do not
    # account for, for now. And that means it needs to be added as an estimated value.
    DEMSim.SetExpandSafetyAdder(2.0)
    # You usually don't have to worry about initial bin size. In very rare cases, init bin size is so bad that auto bin
    # size adaption is effectless, and you should notice in that case kT runs extremely slow. Then in that case setting
    # init bin size may save the simulation.
    # DEMSim.SetInitBinSize(25 * granular_rad);
    DEMSim.SetCDNumStepsMaxDriftMultipleOfAvg(1.2)
    DEMSim.SetCDNumStepsMaxDriftAheadOfAvg(6)
    DEMSim.SetSortContactPairs(True)
    # DEMSim.DisableAdaptiveBinSize();
    DEMSim.SetErrorOutVelocity(1e5)
    # Force the solver to error out if something went crazy. A good practice to add them, but not necessary.
    DEMSim.SetErrorOutAvgContacts(50)
    DEMSim.Initialize()


    # can only be set after initialization 
    # tracker not knowing how many triangles there are 
    mixer_tracker.SetGeometryWildcardValues("Temp", [drum_temperature] * numTriangles)
    mixer_tracker.SetGeometryWildcardValues("Q", [0] * numTriangles)


    sim_end = 5.0
    fps = 20
    frame_time = 1.0 / fps

    # Keep tab of the max velocity in simulation
    max_v_finder = DEMSim.CreateInspector("clump_max_absv")

    print(f"Output at {fps} FPS")
    currframe = 0

    t = 0.
    start = time.process_time()
    while (t < sim_end):
        filename = os.path.join(out_dir, f"DEMdemo_output_{currframe:04d}.csv")
        meshname = os.path.join(out_dir, f"DEMdemo_mesh_{currframe:04d}.vtk")
        DEMSim.WriteSphereFile(filename)
        DEMSim.WriteMeshFile(meshname)
        currframe += 1

        max_v = max_v_finder.GetValue()

        DEMSim.DoDynamics(frame_time)

        Q_values = DEMSim.GetSphereWildcardValue(0, "Q", num_particles)
        T_values = DEMSim.GetSphereWildcardValue(0, "Temp", num_particles)
        conductivity = 100
        for i in range(num_particles):
            T_values[i] += float(Q_values[i]) * frame_time / (float(DEMSim.GetOwnerMass(i)[0]) * conductivity)

        DEMSim.SetSphereWildcardValue(0, "Temp", T_values)
        DEMSim.SetSphereWildcardValue(0, "Q", [0] * num_particles)

        t += frame_time
        print(f"time: {t}, average particle temperature:", np.mean(np.array(T_values)))


    print("average particle temperature:", np.mean(np.array(T_values)))
    elapsed_time = time.process_time() - start
    print(f"{elapsed_time} seconds (wall time) to finish this simulation")
