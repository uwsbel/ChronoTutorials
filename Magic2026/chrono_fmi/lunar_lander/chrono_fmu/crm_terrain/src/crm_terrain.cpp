
#include "crm_terrain.h"

// std::ofstream g_log("C:/temp/exe_log.txt", std::ios::app);
#define LOG(x)                                                                 \
  do {                                                                         \
    std::cout << x << std::endl;                                               \
    fflush(stdout);                                                            \
  } while (0)

using namespace chrono;
// using namespace chrono::vsg3d;
using namespace chrono::vehicle;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
namespace fs = std::filesystem;

int main() {

  setvbuf(stdout, nullptr, _IONBF, 0); // _IONBF = no buffering
  setvbuf(stderr, nullptr, _IONBF, 0);

  LOG("Launched crm_terrain.exe");

  // declare variables
  ChSystemSMC sys;
  ParamsMsg params;
  TerrainLoadMsg terrain_loads_msg;

  // log printing interval
  double next_log_time = 0;

  IntegrationScheme integration_scheme = IntegrationScheme::RK2;

  // set up the system and terrain
  sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81 / 6));
  sys.SetNumThreads(8, 8, 0);
  sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
  sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

  // this is not really needed
  auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
  sys.SetCollisionSystem(collsys);

  // create context and socket for TCP
  // this is the server side that needs to bind to a port
  zmq::context_t ctx(1);
  zmq::socket_t sock(ctx, zmq::socket_type::rep);
  sock.bind("tcp://127.0.0.1:5555");

  // set up the terrain model
  LOG("Waiting for params from FMU");

  // receive initial params
  zmq::message_t msg;
  sock.set(zmq::sockopt::rcvtimeo, 10000);
  auto result = sock.recv(msg, zmq::recv_flags::none);
  memcpy(&params, msg.data(), sizeof(ParamsMsg));

  if (!result) {
    LOG("Params not received before timeout. Killing process!");

    std::cerr << "Params not received before timeout. Killing process!"
              << std::endl;

    return 1;
  }

  // --- do things after receiving parameters ---
  // write to console/terminal or file
  if (params.exe_out_to_file) {

    LOG("Writing to logs at:\n\t" << params.exe_stdout_path << "\n\t"
                                  << params.exe_stderr_path);

    freopen(params.exe_stderr_path, "w", stderr);

    if (freopen(params.exe_stdout_path, "w", stdout) == nullptr) {
      LOG("Failed to redirect stdout to file");
      std::cerr << "Failed to redirect stdout to file\n";
      return 1;
    }
    setvbuf(stdout, nullptr, _IONBF, 0); // _IONBF = no buffering
    setvbuf(stderr, nullptr, _IONBF, 0);
  }

  // logs
  LOG("Params received from FMU");

  LOG("Sim parameters:");
  LOG("\ttimestep: " << params.h);
  LOG("\tcfd_timestep: " << params.h_cfd);
  LOG("\tlog_interval: " << params.log_interval);

  LOG("Tire parameters:");

  LOG("Terrain parameters:");
  LOG("\tpatch_length: " << params.patch_length);
  LOG("\tpatch_width: " << params.patch_width);
  LOG("\tpatch_depth: " << params.patch_depth);

  LOG("\tenable_crm_active_domain: " << params.enable_crm_active_domain);

  LOG("\tactive_box_dim: " << params.active_box_dim);
  LOG("\tactive_domain_settling_time: " << params.active_domain_settling_time);

  LOG("Soil material parameters:");
  LOG("\tdensity: " << params.density);
  LOG("\tcohesion: " << params.cohesion);
  LOG("\tfriction: " << params.friction);
  LOG("\tyoungs_modulus: " << params.youngs_modulus);
  LOG("\tpoisson_ratio: " << params.poisson_ratio);
  LOG("\tnum_proximity_search_steps: " << params.num_proximity_search_steps);
  LOG("\tterrain_spacing: " << params.terrain_spacing);

  LOG("Exe parameters:");
  LOG("\texe_out_to_file: " << params.exe_out_to_file);

  LOG("Visualization parameters:");
  LOG("\tvis_flag: " << params.vis_flag);
  LOG("\tcapture_frames: " << params.capture_frames);
  LOG("\tvis_camera_pos: " << params.vis_camera_pos[0] << ", "
                           << params.vis_camera_pos[1] << ", "
                           << params.vis_camera_pos[2]);
  LOG("\tvis_camera_chase_target: " << params.vis_camera_chase_target);
  LOG("\body_vis: " << params.body_vis);
  LOG("\tmesh_vis: " << params.mesh_vis);
  LOG("\tvis_render_fps: " << params.vis_render_fps);

  LOG("Path parameters: ");
  LOG("\tterrain_save_path: " << params.terrain_save_path);
  LOG("\tfoot_pad_mesh_file_path: " << params.foot_pad_mesh_file_path);
  LOG("\texe_stdout_path: " << params.exe_stdout_path);
  LOG("\texe_stderr_path: " << params.exe_stderr_path);

  // for vis
  double vis_render_time_check = 0;
  double vis_render_freq = params.vis_render_fps;
  double vis_render_time_period = 1 / vis_render_freq;

  // set paths
  chrono::SetChronoDataPath(std::string(params.resource_path) +
                            "/chrono_data/");
  // chrono::SetChronoDataPath(CHRONO_DATA_DIR);
  chrono::vehicle::SetVehicleDataPath(std::string(params.resource_path) +
                                      "/chrono_data/");

  std::string frames_dir = std::string(params.terrain_save_path) + "/frames";
  std::error_code ec;
  fs::remove_all(frames_dir, ec); // delete dir + all old frames (no throw)
  fs::create_directories(frames_dir, ec); // recreate empty

  // mesh paths
  std::string foot_pad_mesh_file_path =
      std::string(params.foot_pad_mesh_file_path);

  auto trimesh =
      ChTriangleMeshConnected::CreateFromWavefrontFile(foot_pad_mesh_file_path);

  LOG("Loaded meshes");

  auto material = chrono_types::make_shared<ChContactMaterialSMC>();

  auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
      material, trimesh, false, false, 0.01);

  auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
  vis_shape->SetMesh(trimesh);
  vis_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));

  std::array<std::shared_ptr<ChBody>, m_PADS> pads;

  //   create tire
  auto make_pad = [&](vehicle::CRMTerrain &terrain,
                      std::shared_ptr<ChBody> &pad, int pad_idx,
                      const ChVector3d &global_pos, const ChQuaterniond &rot,
                      std::string name) {
    // LOG("making pad for index: " << pad_idx);
    pad = chrono_types::make_shared<ChBody>();
    pad->SetMass(0.05);
    pad->SetInertiaXX(ChVector3d(1e-5, 1e-5, 1e-5));
    pad->SetName(name);
    pad->SetPos(global_pos);
    pad->SetRot(rot);

    pad->SetFixed(true);

    if (params.body_vis) {
      pad->AddVisualShape(vis_shape);
    } else {
      // LOG("not adding any visual shape; idx: " << pad_idx);
    }

    // collision shape is added to fsi body
    pad->EnableCollision(false);

    sys.AddBody(pad);

    std::shared_ptr<utils::ChBodyGeometry> pad_geometry;

    pad_geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    pad_geometry->materials.push_back(ChContactMaterialData());

    pad_geometry->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(
        VNULL, QUNIT, foot_pad_mesh_file_path, VNULL));

    terrain.AddRigidBody(
        pad, pad_geometry,
        false); // last argument, check_embedded checks if body mesh starts
                // inside terrain and deletes terrain particles there
  };

  // LOG("after tire lambda function");

  // create terrain
  vehicle::CRMTerrain terrain(sys, params.terrain_spacing);
  auto sysFSI = terrain.GetFsiSystemSPH();
  // LOG("sysFSI raw ptr = " << (void *)sysFSI.get());
  terrain.SetVerbose(true);
  terrain.SetGravitationalAcceleration(
      ChVector3d(0, 0, -9.81 / 6)); // in the -Z direction
  terrain.SetStepSizeCFD(params.h_cfd);
  terrain.SetStepsizeMBD(params.h);

  LOG("CRMTerrain created");

  //   create tires based on num_tires
  for (int i = 0; i < m_PADS; i++) {
    // send global pos to the function
    make_pad(terrain, pads[i], i, VNULL, QUNIT, "pad_" + std::to_string(i));
  }

  // Set SPH parameters and soil material properties
  ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
  mat_props.density = params.density;
  mat_props.Young_modulus = params.youngs_modulus;
  mat_props.Poisson_ratio = params.poisson_ratio;
  mat_props.mu_I0 = 0.04;
  mat_props.mu_fric_s = params.friction;
  mat_props.mu_fric_2 = params.friction;
  mat_props.average_diam = 0.005;
  mat_props.cohesion_coeff = params.cohesion;
  terrain.SetElasticSPH(mat_props);

  LOG("SetElasticSPH done");

  // Set SPH solver parameters
  ChFsiFluidSystemSPH::SPHParameters sph_params;
  sph_params.integration_scheme = integration_scheme;
  sph_params.initial_spacing = params.terrain_spacing;
  sph_params.d0_multiplier = 1.0;
  sph_params.free_surface_threshold = 2.0;
  sph_params.artificial_viscosity = 0.5;
  sph_params.use_consistent_gradient_discretization = false;
  sph_params.use_consistent_laplacian_discretization = false;
  sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
  sph_params.boundary_method = BoundaryMethod::ADAMI;
  sph_params.num_proximity_search_steps = 2;
  terrain.SetSPHParameters(sph_params);

  LOG("SetSPHParameters done");

  // --- send back to fmu
  // this is a formality; in case exe crashes, fmu zmq rcv timeout will
  // terminate the fmu
  int8_t tcp_ack = 0;
  zmq::message_t init_ack_msg_to_send(sizeof(tcp_ack));
  memcpy(init_ack_msg_to_send.data(), &tcp_ack, sizeof(tcp_ack));
  sock.send(init_ack_msg_to_send, zmq::send_flags::none);

  // initialize internal time
  double m_time = 0; // initialize time at 0

  // --- one time
  sock.set(zmq::sockopt::rcvtimeo, 10000);
  zmq::message_t reqmsg;
  result = sock.recv(reqmsg,
                     zmq::recv_flags::none); // this is a blocking wait function

  StateMsg state_in;
  memcpy(&state_in, reqmsg.data(), sizeof(StateMsg));

  for (int i = 0; i < m_PADS; i++) {
    AssignStates(pads[i], i, state_in);
  }

  // set active domains
  if (params.enable_crm_active_domain) {
    terrain.SetActiveDomain(ChVector3d(params.active_box_dim));
    terrain.SetActiveDomainDelay(params.active_domain_settling_time);
    LOG("Active domain set");
  }

  terrain.Construct(ChVector3d(params.patch_length, params.patch_width,
                               params.patch_depth), // length X width X height
                    ChVector3d(0, 0,
                               -params.patch_depth), // patch top at z = 0
                    BoxSide::ALL &
                        ~BoxSide::Z_POS); // all boundaries, except top

  LOG("Fixed patch constructed");

  // Initialize the terrain system
  terrain.Initialize();

  auto aabb = terrain.GetSPHBoundingBox(); // ChAABB in absolute frame
  LOG("[terrain] SPH AABB min=(" << aabb.min.x() << ", " << aabb.min.y() << ", "
                                 << aabb.min.z() << ")  max=(" << aabb.max.x()
                                 << ", " << aabb.max.y() << ", " << aabb.max.z()
                                 << ")\n");

  LOG("Terrain and rigid body setup complete!");

  // setting up vis

  std::shared_ptr<chrono::vsg3d::ChVisualSystemVSG> vis;
  if (params.vis_flag) {
    // LOG("vis: creating plugin"); // A
    auto visFSI =
        chrono_types::make_shared<chrono::fsi::sph::ChSphVisualizationVSG>(
            sysFSI.get());
    // LOG("vis: plugin created"); // B

    visFSI->EnableFluidMarkers(true);
    visFSI->EnableBoundaryMarkers(false);
    visFSI->EnableRigidBodyMarkers(params.mesh_vis);

    auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(
        aabb.min.z(), aabb.max.z());
    visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::BROWN);
    // LOG("vis: markers configured"); // C

    vis = chrono_types::make_shared<chrono::vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    // LOG("vis: system created"); // D
    vis->AttachPlugin(visFSI);
    // LOG("vis: plugin attached"); // E
    vis->SetWindowTitle("CRM external process");
    vis->SetWindowSize(1280, 720);
    vis->AddCamera(chrono::ChVector3d(params.vis_camera_pos[0],
                                      params.vis_camera_pos[1],
                                      params.vis_camera_pos[2]),
                   chrono::ChVector3d(0, 0, 0));
    vis->SetCameraVertical(chrono::CameraVerticalDir::Z);
    // LOG("vis: about to Initialize"); // F
    // vsg::Logger::instance()->level = vsg::Logger::LOGGER_DEBUG;
    vis->Initialize();
    // LOG("vis: initialized"); // G

    if (params.capture_frames) {
      vis->SetImageOutputDirectory(frames_dir);
      vis->SetImageOutput(true);
    }
  }

  terrain_loads_msg.save_results_to_file_ack = -1; // default value

  if (state_in.step_size > params.h) {
    LOG("Step size larger than internal timestep; aborting!"
        << "\tStep size: " << state_in.step_size);
    return 1;
  }

  // advance terrain
  terrain.Synchronize(sys.GetChTime());
  terrain.DoStepDynamics(state_in.step_size);

  if (vis) {
    if (!vis->Run()) { // false when the window is closed
      vis.reset();
      vis = nullptr; // stop rendering, keep simulating
    } else {

      if (params.vis_camera_chase_target)
        vis->SetCameraTarget((pads[0]->GetPos() + pads[2]->GetPos()) / 2);

      vis->Render(); // draws one frame; writes a PNG if
                     // SetImageOutput(true)
    }
  }

  m_time += state_in.step_size;

  // get forces
  for (int i = 0; i < m_PADS; i++) {
    GetTerrainLoads(terrain_loads_msg, pads[i], i, terrain);
  }

  if (m_time >= next_log_time) {

    LOG(stateLine(m_time, params, state_in, terrain_loads_msg));

    next_log_time += params.log_interval;
  }

  zmq::message_t terrain_loads_msg_to_send(sizeof(TerrainLoadMsg));
  memcpy(terrain_loads_msg_to_send.data(), &terrain_loads_msg,
         sizeof(TerrainLoadMsg));
  sock.send(terrain_loads_msg_to_send, zmq::send_flags::none);

  // --- do in a loop
  while (1) {
    zmq::message_t reqmsg;
    result =
        sock.recv(reqmsg,
                  zmq::recv_flags::none); // this is a blocking wait function

    StateMsg state_in;
    memcpy(&state_in, reqmsg.data(), sizeof(StateMsg));

    if (!state_in.save_results_to_file_flag) {
      for (int i = 0; i < m_PADS; i++) {
        AssignStates(pads[i], i, state_in);
      }

      if (state_in.step_size > params.h) {
        LOG("Step size larger than internal timestep; aborting!"
            << "\tStep size: " << state_in.step_size);
        return 1;
      }

      // advance terrain
      terrain.Synchronize(sys.GetChTime());
      terrain.DoStepDynamics(state_in.step_size);

      if (vis) {
        if (!vis->Run()) { // false when the window is closed
          vis.reset();
          vis = nullptr; // stop rendering, keep simulating
        } else {
          if (m_time > vis_render_time_check) {

            if (params.vis_camera_chase_target)
              vis->SetCameraTarget((pads[0]->GetPos() + pads[2]->GetPos()) / 2);

            vis->Render(); // draws one frame; writes a PNG if
            // SetImageOutput(true)

            vis_render_time_check += vis_render_time_period;
          }
        }
      }

      m_time += state_in.step_size;

      // get forces
      for (int i = 0; i < m_PADS; i++) {
        GetTerrainLoads(terrain_loads_msg, pads[i], i, terrain);
      }

      // logs
      if (m_time >= next_log_time) {

        LOG(stateLine(m_time, params, state_in, terrain_loads_msg));

        next_log_time += params.log_interval;
      }
    } else {

      if (vis)
        vis = nullptr; // destroy before GPU/FSI teardown

      terrain.SaveOutputData(sys.GetChTime(), params.terrain_save_path,
                             params.terrain_save_path);

      terrain_loads_msg.save_results_to_file_ack = 0; // success

      LOG("Waiting for particle data to finish writing...\n");
      std::this_thread::sleep_for(std::chrono::seconds(120));

      LOG("CRM terrain files wrtitten to: " << params.terrain_save_path
                                            << "\n");
    }

    zmq::message_t terrain_loads_msg_to_send(sizeof(TerrainLoadMsg));
    memcpy(terrain_loads_msg_to_send.data(), &terrain_loads_msg,
           sizeof(TerrainLoadMsg));
    sock.send(terrain_loads_msg_to_send, zmq::send_flags::none);
  }
}

void AssignStates(std::shared_ptr<ChBody> &tire, int tire_idx,
                  const StateMsg &state_in_msg) {
  WheelState wheel_state;

  wheel_state.pos =
      ChVector3d(state_in_msg.pos[tire_idx][0], state_in_msg.pos[tire_idx][1],
                 state_in_msg.pos[tire_idx][2]);

  wheel_state.rot = ChQuaterniond(
      state_in_msg.rot[tire_idx][0], state_in_msg.rot[tire_idx][1],
      state_in_msg.rot[tire_idx][2], state_in_msg.rot[tire_idx][3]);

  // check and correct quaternion norm
  int status = CheckQuatNorm(wheel_state.rot);
  if (status != 0) {
    std::cerr << "Fatal error in CheckQuatNorm\n";
    std::cout << "Fatal error in CheckQuatNorm\n";
    std::exit(1); // terminates the whole program immediately, with exit code 1
  }

  wheel_state.lin_vel = ChVector3d(state_in_msg.lin_vel[tire_idx][0],
                                   state_in_msg.lin_vel[tire_idx][1],
                                   state_in_msg.lin_vel[tire_idx][2]);

  wheel_state.ang_vel = ChVector3d(state_in_msg.ang_vel[tire_idx][0],
                                   state_in_msg.ang_vel[tire_idx][1],
                                   state_in_msg.ang_vel[tire_idx][2]);

  tire->SetPos(wheel_state.pos);
  tire->SetRot(wheel_state.rot);
  tire->SetLinVel(wheel_state.lin_vel);
  tire->SetAngVelParent(wheel_state.ang_vel);
}

void GetTerrainLoads(TerrainLoadMsg &terrain_loads_msg,
                     const std::shared_ptr<ChBody> &tire, int tire_idx,
                     const CRMTerrain &terrain) {
  terrain_loads_msg.force[tire_idx][0] = terrain.GetFsiBodyForce(tire).x();
  terrain_loads_msg.force[tire_idx][1] = terrain.GetFsiBodyForce(tire).y();
  terrain_loads_msg.force[tire_idx][2] = terrain.GetFsiBodyForce(tire).z();

  terrain_loads_msg.moment[tire_idx][0] = terrain.GetFsiBodyTorque(tire).x();
  terrain_loads_msg.moment[tire_idx][1] = terrain.GetFsiBodyTorque(tire).y();
  terrain_loads_msg.moment[tire_idx][2] = terrain.GetFsiBodyTorque(tire).z();
}

int CheckQuatNorm(ChQuaterniond &quat) {
  double norm = quat.Length();

  if (std::abs(norm - 1) > 1e-2) {
    LOG("Quaternion norm out of expected bounds!");
    return 1;
  } else {
    quat.Normalize();
  }

  return 0;
}

std::string stateLine(double time, const ParamsMsg &params,
                      const StateMsg &state_msg,
                      const TerrainLoadMsg &terrain_loads_msg) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4);
  ss << "t=" << time;
  for (int i = 0; i < m_PADS; i++) {
    ss << "\ntire " << i << "\n "
       << "\tpos=(" << state_msg.pos[i][0] << ", " << state_msg.pos[i][1]
       << ", " << state_msg.pos[i][2] << ")"
       << "\trot=(" << state_msg.rot[i][0] << ", " << state_msg.rot[i][1]
       << ", " << state_msg.rot[i][2] << ", " << state_msg.rot[i][3] << ")"
       << "\n"
       << "\tlinvel=(" << state_msg.lin_vel[i][0] << ", "
       << state_msg.lin_vel[i][1] << ", " << state_msg.lin_vel[i][2] << ")"
       << "\tangvel=(" << state_msg.ang_vel[i][0] << ", "
       << state_msg.ang_vel[i][1] << ", " << state_msg.ang_vel[i][2] << ")"
       << "\n"
       << "\tFx=" << terrain_loads_msg.force[i][0]
       << "\tFy=" << terrain_loads_msg.force[i][1]
       << "\tFz=" << terrain_loads_msg.force[i][2] << "\n"
       << "\tMx=" << terrain_loads_msg.moment[i][0]
       << "\tMy=" << terrain_loads_msg.moment[i][1]
       << "\tMz=" << terrain_loads_msg.moment[i][2];
  }
  return ss.str();
}
