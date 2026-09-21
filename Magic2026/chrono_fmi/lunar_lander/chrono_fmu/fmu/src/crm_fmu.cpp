// =============================================================================
// Authors: Ahmed Ansari
// UW-Madison
// Simulation Based Engineering Lab (SBEL)
// =============================================================================
// Chrono FMU for CRM terrain
// To be used with Simscape

#include "crm_fmu.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::fmi3;

// Create an instance of this FMU
fmu_forge::fmi3::FmuComponentBase *fmu_forge::fmi3::fmi3InstantiateIMPL(
    fmu_forge::fmi3::FmuType fmiInterfaceType, fmi3String instanceName,
    fmi3String instantiationToken, fmi3String resourcePath, fmi3Boolean visible,
    fmi3Boolean loggingOn, fmi3InstanceEnvironment instanceEnvironment,
    fmi3LogMessageCallback logMessage) {
  return new FmuComponent(fmiInterfaceType, instanceName, instantiationToken,
                          resourcePath, visible, loggingOn, instanceEnvironment,
                          logMessage);
}

// -----------------------------------------------------------------------------

FmuComponent::FmuComponent(fmu_forge::fmi3::FmuType fmiInterfaceType,
                           fmi3String instanceName,
                           fmi3String instantiationToken,
                           fmi3String resourcePath, fmi3Boolean visible,
                           fmi3Boolean loggingOn,
                           fmi3InstanceEnvironment instanceEnvironment,
                           fmi3LogMessageCallback logMessage)
    : FmuChronoComponentBase(fmiInterfaceType, instanceName, instantiationToken,
                             resourcePath, visible, loggingOn,
                             instanceEnvironment, logMessage) {
  // Initialize FMU type
  initializeType(fmiInterfaceType);

  // std::ofstream logf("C:/temp/fmu_ctor.txt", std::ios::app);
  // logf << "constructor entered" << std::endl;
  // logf.flush();

  // for fmu logging
  m_current_time = 0;
  m_next_log_time = 0;
  m_sim_end_time = -1;

  // Set start values for FMU input and output variables (just in case)
  // params default values

  // just in case

  params.patch_length = 10.0; // x: ~2.5× wheelbase, room fore/aft
  params.patch_width = 10.0;  // y: 3 m track + 1.5 m each side
  params.patch_depth = 0.5;   // z: ample sinkage depth (could trim to 0.7)

  params.active_box_dim = 1;
  params.active_domain_settling_time = 0;

  // active domain and moving patch
  params.enable_crm_active_domain = true;

  // CRM material properties
  params.density = 1700;
  params.cohesion = 5e3;
  params.friction = 0.8;
  params.youngs_modulus = 1e6;
  params.poisson_ratio = 0.3;

  params.num_proximity_search_steps = 2;

  params.terrain_spacing = 4e-2;

  m_save_terrain_flag = false;

  m_terrain_save_path = ""; // blank and need to be overwritten
  m_foot_pad_mesh_file_path = "";

  std::memset(params.terrain_save_path, 0,
              sizeof(params.terrain_save_path)); // zero-fill

  std::strncpy(params.terrain_save_path, m_terrain_save_path.c_str(),
               sizeof(params.terrain_save_path) - 1);

  std::memset(params.foot_pad_mesh_file_path, 0,
              sizeof(params.foot_pad_mesh_file_path)); // zero-fill
  std::strncpy(params.foot_pad_mesh_file_path,
               m_foot_pad_mesh_file_path.c_str(),
               sizeof(params.foot_pad_mesh_file_path) - 1);

  //  sim parameters
  params.h = 2e-4;
  params.h_cfd = 1e-5;
  params.log_interval = 0.5;
  params.vis_flag = false;       // visualization off by default
  params.capture_frames = false; // captuure frames for video

  // --- Set start default values for FMU inputs ---
  // initialize as 0/null vectors
  for (int i = 0; i < m_PADS; i++) {
    // initialize as 0
    m_states_array[i] = Eigen::VectorXd::Zero(m_STATE_SIZE);

    m_states_array[i](2) = 2.04; // no use since we're now reading pos directly
                                 // from actual states before creating terrain
    m_states_array[i](3) = 1;    // quaternion w
  }

  // --- Set start default values for FMU outputs ---
  // initialize as 0/null vectors
  for (int i = 0; i < m_PADS; i++) {
    m_terrain_loads_array[i] = Eigen::VectorXd::Zero(m_TERRAIN_LOAD_SIZE);
  }

  states_msg.current_time = 0;
  states_msg.communication_step_size = 0;
  states_msg.step_size = 0;
  states_msg.save_results_to_file_flag = false;

  terrain_loads_msg.save_results_to_file_ack = -1;

  // exe path
  work_dir = std::string(resourcePath);
  exe_path = work_dir + "/crm_terrain.exe";

  // with:
  if (resourcePath != nullptr) {
    work_dir = std::string(resourcePath);
    exe_path = work_dir + "/crm_terrain.exe";
  } else {
    work_dir.clear();
    exe_path.clear();
  }

  // exe stdout path
  params.exe_out_to_file = false;
  m_exe_stdout_path = "";
  m_exe_stderr_path = "";

  std::memset(params.exe_stdout_path, 0,
              sizeof(params.exe_stdout_path)); // zero-fill
  std::strncpy(params.exe_stdout_path, m_exe_stdout_path.c_str(),
               sizeof(params.exe_stdout_path) - 1);

  std::memset(params.exe_stderr_path, 0,
              sizeof(params.exe_stderr_path)); // zero-fill
  std::strncpy(params.exe_stderr_path, m_exe_stderr_path.c_str(),
               sizeof(params.exe_stderr_path) - 1);

  // vis params
  m_vis_camera_pos = ChVector3d(-10, -20, 20);

  params.vis_camera_pos[0] = m_vis_camera_pos.x();
  params.vis_camera_pos[1] = m_vis_camera_pos.y();
  params.vis_camera_pos[2] = m_vis_camera_pos.z();

  params.vis_camera_chase_target = false;
  params.body_vis = true;
  params.mesh_vis = false;
  params.vis_render_fps = 30;

  // add fmu variables
  // add unit definition for inertia
  addUnitDefinition(fmu_forge::UnitDefinition("kgm2", 1, 2, 0, 0, 0, 0, 0, 0));

  // // sim settings
  AddFmuVariable(&params.h, "sim_params.timestep", FmuVariable::Type::Float64,
                 "s", "timestep", FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(
      &params.h_cfd, "sim_params.cfd_timestep", FmuVariable::Type::Float64, "s",
      "cfd timestep", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  AddFmuVariable(&params.log_interval, "sim_params.log_interval",
                 FmuVariable::Type::Float64, "s", "log_interval",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(
      &m_sim_end_time, "sim_params.sim_end_time", FmuVariable::Type::Float64,
      "s", "sim end time", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  // terrain params
  AddFmuVariable(&params.patch_length, "terrain_params.patch_length",
                 FmuVariable::Type::Float64, "m", "patch length",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(&params.patch_width, "terrain_params.patch_width",
                 FmuVariable::Type::Float64, "m", "patch width",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(&params.patch_depth, "terrain_params.patch_depth",
                 FmuVariable::Type::Float64, "m", "patch depth",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(
      &params.enable_crm_active_domain,
      "terrain_params.enable_crm_active_domain", FmuVariable::Type::Boolean,
      "1", "enable crm active domain", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  AddFmuVariable(&params.active_box_dim, "terrain_params.active_box_dim",
                 FmuVariable::Type::Float64, "m", "active box dim",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(
      &params.active_domain_settling_time,
      "terrain_params.active_domain_settling_time", FmuVariable::Type::Float64,
      "s", "active domain settling time", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  // soil materials
  AddFmuVariable(&params.density, "terrain_params.material_density",
                 FmuVariable::Type::Float64, "1", "material density",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(&params.cohesion, "terrain_params.material_cohesion",
                 FmuVariable::Type::Float64, "1", "material cohesion",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(&params.friction, "terrain_params.material_friction",
                 FmuVariable::Type::Float64, "1", "material friction",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(
      &params.youngs_modulus, "terrain_params.material_youngs_modulus",
      FmuVariable::Type::Float64, "1", "material youngs modulus",
      FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  AddFmuVariable(&params.poisson_ratio, "terrain_params.material_poisson_ratio",
                 FmuVariable::Type::Float64, "1", "material poisson ratio",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(
      &params.num_proximity_search_steps,
      "terrain_params.num_proximity_search_steps", FmuVariable::Type::Int32,
      "1", "num proximity search steps", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  AddFmuVariable(&params.terrain_spacing, "terrain_params.terrain_spacing",
                 FmuVariable::Type::Float64, "m", "terrain spacing",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(&m_save_terrain_flag, "terrain_params.save_terrain_flag",
                 FmuVariable::Type::Boolean, "1", "save terrain flag",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  //  vis params
  AddFmuVariable(
      &params.vis_flag, "vis_params.vis_flag", FmuVariable::Type::Boolean, "1",
      "vis flag", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  AddFmuVariable(&params.capture_frames, "vis_params.capture_frames",
                 FmuVariable::Type::Boolean, "1", "capture video frames flag",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVecVariable(m_vis_camera_pos, "vis_params.camera_pos", "1",
                    "camera position for visualization",
                    FmuVariable::CausalityType::parameter,
                    FmuVariable::VariabilityType::fixed,
                    FmuVariable::InitialType::exact);

  AddFmuVariable(
      &params.vis_camera_chase_target, "vis_params.camera_chase_target",
      FmuVariable::Type::Boolean, "1", "camera chase target",
      FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  AddFmuVariable(
      &params.body_vis, "vis_params.body_vis", FmuVariable::Type::Boolean, "1",
      "visualize foot_pad body", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  AddFmuVariable(
      &params.mesh_vis, "vis_params.mesh_vis", FmuVariable::Type::Boolean, "1",
      "visualize foot_pad mesh", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  AddFmuVariable(&params.vis_render_fps, "vis_params.render_fps",
                 FmuVariable::Type::Float64, "1", "visualize fps",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  //  exe file write
  AddFmuVariable(&params.exe_out_to_file, "exe_params.write_logs_to_file",
                 FmuVariable::Type::Boolean, "1", "write logs to file",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  //  path vars
  // terrain save path
  AddFmuVariable(&m_terrain_save_path, "paths.terrain_save_path",
                 FmuVariable::Type::String, "1", "terrain save path",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(&m_foot_pad_mesh_file_path, "paths.foot_pad_mesh_file_path",
                 FmuVariable::Type::String, "1",
                 "left foot_pad mesh path (absolute/full file path)",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed,
                 FmuVariable::InitialType::exact);

  AddFmuVariable(
      &m_exe_stdout_path, "paths.exe_stdout_path", FmuVariable::Type::String,
      "1", "exe stderr path", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  AddFmuVariable(
      &m_exe_stderr_path, "paths.exe_stderr_path", FmuVariable::Type::String,
      "1", "exe stderr path", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed, FmuVariable::InitialType::exact);

  // --- FMU inputs ---
  for (int i = 0; i < m_PADS; i++) {

    AddFmuVariable(m_states_array[i].data(),
                   "wheel_" + std::to_string(i) + "_state",
                   FmuVariable::Type::Float64, {{m_STATE_SIZE, true}}, "1",
                   "pad " + std::to_string(i) + " state",
                   FmuVariable::CausalityType::input,
                   FmuVariable::VariabilityType::continuous,
                   FmuVariable::InitialType::exact);
  }

  // fmu outputs
  for (int i = 0; i < m_PADS; i++) {

    AddFmuVariable(m_terrain_loads_array[i].data(),
                   "terrain_" + std::to_string(i) + "_loads",
                   FmuVariable::Type::Float64, {{m_TERRAIN_LOAD_SIZE, true}},
                   "1", "terrain " + std::to_string(i) + " loads",
                   FmuVariable::CausalityType::output,
                   FmuVariable::VariabilityType::continuous,
                   FmuVariable::InitialType::exact);
  }

  // Specify functions to process input variables (at beginning of step)
  AddPreStepFunction([this]() { this->ProcessInputs(); });

  // Specify functions to calculate FMU outputs (at end of step)
  AddPostStepFunction([this]() { this->ProcessOutputs(); });

  // logf << "end of constructor" << std::endl;
  // logf.flush();
}

// definition for processing inputs
void FmuComponent::ProcessInputs() {

  // convert chrono pad states to StateMsg struct
  ChronoStates2POD();
}

// defintion for calculating outputs
void FmuComponent::ProcessOutputs() {

  // change struct back to chrono terrain force types
  POD2ChronoTerrainLoads();
  logData();
}

// this is run before creating the model description at build time
void FmuComponent::preModelDescriptionExport() {}

// this is run after creating the model description at build time
void FmuComponent::postModelDescriptionExport() {}

// this is run before receiving parameters from the master
fmi3Status FmuComponent::enterInitializationModeIMPL() {

  // std::ofstream logf("C:/temp/fmu_ctor.txt", std::ios::app);
  // logf << "enterinit entered" << std::endl;
  // logf.flush();

  return fmi3Status::fmi3OK;
}

// this is run after receiving the parameters from master
fmi3Status FmuComponent::exitInitializationModeIMPL() {
  // std::ofstream logf("C:/temp/fmu_log.txt", std::ios::app);
  // logf << "exitInit reached. work_dir=[" << work_dir << "] exe_path=["
  //      << exe_path << "]" << std::endl;
  // logf.flush();

  logTagged("***Starting Chrono SCM Terrain FMU!***", fmi3Status::fmi3OK,
            "logAll");

  // --- populate the params message struct for chrono vars and strings ---

  // strings
  std::memset(params.resource_path, 0,
              sizeof(params.resource_path)); // zero-fill
  std::strncpy(params.resource_path, work_dir.c_str(),
               sizeof(params.resource_path) - 1);

  std::memset(params.terrain_save_path, 0,
              sizeof(params.terrain_save_path)); // zero-fill
  std::strncpy(params.terrain_save_path, m_terrain_save_path.c_str(),
               sizeof(params.terrain_save_path) - 1);

  std::memset(params.foot_pad_mesh_file_path, 0,
              sizeof(params.foot_pad_mesh_file_path)); // zero-fill
  std::strncpy(params.foot_pad_mesh_file_path,
               m_foot_pad_mesh_file_path.c_str(),
               sizeof(params.foot_pad_mesh_file_path) - 1);

  std::memset(params.exe_stdout_path, 0,
              sizeof(params.exe_stdout_path)); // zero-fill
  std::strncpy(params.exe_stdout_path, m_exe_stdout_path.c_str(),
               sizeof(params.exe_stdout_path) - 1);

  std::memset(params.exe_stderr_path, 0,
              sizeof(params.exe_stderr_path)); // zero-fill
  std::strncpy(params.exe_stderr_path, m_exe_stderr_path.c_str(),
               sizeof(params.exe_stderr_path) - 1);

  //  chrono to POD camera pos
  params.vis_camera_pos[0] = m_vis_camera_pos.x();
  params.vis_camera_pos[1] = m_vis_camera_pos.y();
  params.vis_camera_pos[2] = m_vis_camera_pos.z();

  //  capture frames is false if vis is false
  if (!params.vis_flag)
    params.capture_frames = false;
  //  -------------------------------------------------------

  // --- start external process ---
  LaunchSolver(exe_path);
  logTagged("Launched external crm terrain from " + exe_path,
            fmi3Status::fmi3OK, "logAll");
  // add sleep here
  // wait and get acknowledgement from external process
  // this is done by the message receive

  WaitForSingleObject(m_procInfo.hProcess, 1000); // wait up to 1 s

  DWORD exitCode;
  if (GetExitCodeProcess(m_procInfo.hProcess, &exitCode)) {
    if (exitCode != STILL_ACTIVE) {
      // process already exited => it crashed on startup
      std::stringstream ss;
      ss << "0x" << std::hex << std::uppercase << exitCode;
      std::string msg =
          "External solver died on startup, exit code " + ss.str();
      if (exitCode == 0xC0000135)
        msg += " (STATUS_DLL_NOT_FOUND - a required DLL is missing, e.g. "
               "vsg-17.dll)";
      logTagged(msg, fmi3Status::fmi3Fatal, "logAll");
      return fmi3Status::fmi3Fatal; // fail fast, don't block on recv
    }
  } else {
    logTagged("GetExitCodeProcess failed: " + std::to_string(GetLastError()),
              fmi3Status::fmi3Fatal, "logAll");
    return fmi3Status::fmi3Fatal;
  }

  // create TCP socket
  ctx = zmq::context_t(1);
  sock = zmq::socket_t(ctx, zmq::socket_type::req);
  sock.connect("tcp://127.0.0.1:5555");

  logTagged("Set up the socket to connect to external process",
            fmi3Status::fmi3OK, "logAll");

  // pass parameters to external process
  zmq::message_t params_msg(sizeof(ParamsMsg));
  memcpy(params_msg.data(), &params, sizeof(ParamsMsg));
  sock.send(params_msg, zmq::send_flags::none);

  // get initial forces/acknowledgement for params received; timeout of 10
  // seconds

  logTagged("Waiting for acknowledgement from external process...",
            fmi3Status::fmi3OK, "logAll");

  // external process only sends 0 (success) or nothing
  int tcp_ack;
  sock.set(zmq::sockopt::rcvtimeo,
           300000); // 100 seconds for the initial send-recv
  zmq::message_t tcp_ack_msg(sizeof(tcp_ack));
  auto result = sock.recv(tcp_ack_msg, zmq::recv_flags::none);

  // convert to struct type
  memcpy(&tcp_ack, tcp_ack_msg.data(), sizeof(tcp_ack));

  if (!result || !tcp_ack) {
    logTagged("Acknowledgement not received from external process.\n"
              "Terminating FMU!",
              fmi3Status::fmi3Fatal, "logStatusError");

    TerminateExternalProcess();

    return fmi3Status::fmi3Fatal;
  }

  logTagged("Received acknowledgement from external process",
            fmi3Status::fmi3OK, "logAll");

  // reduce timeout to 10s for each timestep send-recv
  sock.set(zmq::sockopt::rcvtimeo, 10000);

  logTagged("exitInit: Initialization okay!", fmi3Status::fmi3OK, "logAll");

  return fmi3Status::fmi3OK;
}

// for running the FMU dynamics
fmi3Status FmuComponent::doStepIMPL(
    fmi3Float64 currentCommunicationPoint, fmi3Float64 communicationStepSize,
    fmi3Boolean noSetFMUStatePriorToCurrentPoint,
    fmi3Boolean *eventHandlingNeeded, fmi3Boolean *terminateSimulation,
    fmi3Boolean *earlyReturn, fmi3Float64 *lastSuccessfulTime) {

  // Advance FMU state to next communication point
  // throw error if comm step size is grater than +1% of internal timestep
  if (communicationStepSize - params.h > (params.h / 100)) {
    logTagged("doStepIMPL: comm step size for the FMU cannot exceed the "
              "internal timstep.",
              fmi3Status::fmi3Error, "logStatusError");
    return fmi3Status::fmi3Error;
  }

  while (m_time < currentCommunicationPoint + communicationStepSize) {
    fmi3Float64 step_size = std::min(communicationStepSize, params.h);

    states_msg.current_time = m_time;
    states_msg.communication_step_size = communicationStepSize;
    states_msg.step_size = step_size;

    // send and receive data from external crm exe
    zmq::message_t state_msg_send(sizeof(StateMsg));
    memcpy(state_msg_send.data(), &states_msg, sizeof(StateMsg));
    sock.send(state_msg_send, zmq::send_flags::none);

    // get back terrain loads
    sock.set(zmq::sockopt::rcvtimeo, 10000);
    zmq::message_t terrain_loads_msg_recv(sizeof(TerrainLoadMsg));
    auto result = sock.recv(terrain_loads_msg_recv, zmq::recv_flags::none);

    // convert to struct type
    memcpy(&terrain_loads_msg, terrain_loads_msg_recv.data(),
           sizeof(TerrainLoadMsg));

    if (!result) {
      logTagged(
          "Terrain forces not received from external solver. Terminating FMU!",
          fmi3Status::fmi3Fatal, "logAll");
      return fmi3Status::fmi3Fatal;
    }

    m_time += step_size;
  }

  return fmi3Status::fmi3OK;
}

// launching external process
bool FmuComponent::LaunchSolver(const std::string &exe_path_arg) {
  STARTUPINFOA si = {};
  si.cb = sizeof(si);
  ZeroMemory(&m_procInfo, sizeof(m_procInfo));

  std::string cmdline = exe_path_arg;
  std::vector<char> cmd(cmdline.begin(), cmdline.end());
  cmd.push_back('\0');

  BOOL ok = CreateProcessA(nullptr, cmd.data(), nullptr, nullptr, FALSE, 0,
                           nullptr, work_dir.c_str(), &si, &m_procInfo);

  if (!ok) {
    logTagged("CreateProcess failed: " + std::to_string(GetLastError()),
              fmi3Status::fmi3Fatal, "logAll");
    return false;
  }

  // Give the process a moment to either start up or crash (e.g., missing DLL)
  WaitForSingleObject(m_procInfo.hProcess, 500); // wait up to 0.5 s

  // Check if it already died
  DWORD exitCode;
  if (GetExitCodeProcess(m_procInfo.hProcess, &exitCode)) {
    if (exitCode != STILL_ACTIVE) {
      // process exited already → it crashed on startup
      std::string msg = "Solver exited immediately with code 0x" +
                        /* hex format */ std::to_string(exitCode);
      if (exitCode == 0xC0000135)
        msg += " (STATUS_DLL_NOT_FOUND — a required DLL is missing, "
               "likely compute.dll not next to the exe)";
      logTagged(msg, fmi3Status::fmi3Fatal, "logAll");
      return false;
    }
  }

  // still running — good
  return true;
}

// destructor for destroying external process
FmuComponent::~FmuComponent() {
  try {
    if (m_save_terrain_flag) {
      if (m_time > (m_sim_end_time - params.h)) {

        logTagged("destructor: Writing terrain results to file ...",
                  fmi3Status::fmi3OK, "logAll");

        states_msg.save_results_to_file_flag = true;

        zmq::message_t state_msg_send(sizeof(StateMsg));
        memcpy(state_msg_send.data(), &states_msg, sizeof(StateMsg));
        sock.send(state_msg_send, zmq::send_flags::none);

        // get back write terrain ack
        sock.set(zmq::sockopt::rcvtimeo, 300000);
        zmq::message_t terrain_loads_msg_recv(sizeof(TerrainLoadMsg));
        auto result = sock.recv(terrain_loads_msg_recv, zmq::recv_flags::none);

        // convert to struct type
        memcpy(&terrain_loads_msg, terrain_loads_msg_recv.data(),
               sizeof(TerrainLoadMsg));

        if (!result || terrain_loads_msg.save_results_to_file_ack) {
          logTagged("destructor: Could not write terrain results to file",
                    fmi3Status::fmi3OK, "logAll");
        } else {
          logTagged("destructor: Terrain results successfully written to: " +
                        std::string(params.terrain_save_path),
                    fmi3Status::fmi3OK, "logAll");
        }

      } else {
        logTagged("destructor: Sim run time smaller than end time. Not writing "
                  "terrain results "
                  "to file.",
                  fmi3Status::fmi3OK, "logAll");
      }

    } else {
      logTagged("destructor: Terrain write disabled. Not writing terrain "
                "results to file.",
                fmi3Status::fmi3OK, "logAll");
    }

  } catch (...) {
    // do nothing
  }

  TerminateExternalProcess();
}

void FmuComponent::logTagged(std::string msg, fmi3Status status,
                             std::string msg_cat) {
  sendToLog("[" + this->m_instanceName + "] " + msg, status, msg_cat);
}

// convert recvd chrono pad states to StateMsg struct
void FmuComponent::ChronoStates2POD(void) {

  for (int i = 0; i < m_PADS; i++) {
    states_msg.pos[i][0] = m_states_array[i](0);
    states_msg.pos[i][1] = m_states_array[i](1);
    states_msg.pos[i][2] = m_states_array[i](2);

    states_msg.rot[i][0] = m_states_array[i](3);
    states_msg.rot[i][1] = m_states_array[i](4);
    states_msg.rot[i][2] = m_states_array[i](5);
    states_msg.rot[i][3] = m_states_array[i](6);

    states_msg.lin_vel[i][0] = m_states_array[i](7);
    states_msg.lin_vel[i][1] = m_states_array[i](8);
    states_msg.lin_vel[i][2] = m_states_array[i](9);

    states_msg.ang_vel[i][0] = m_states_array[i](10);
    states_msg.ang_vel[i][1] = m_states_array[i](11);
    states_msg.ang_vel[i][2] = m_states_array[i](12);
  }
}

// convert recvd chrono pad states to StateMsg struct
void FmuComponent::POD2ChronoTerrainLoads(void) {

  for (int i = 0; i < m_PADS; i++) {

    m_terrain_loads_array[i](0) = terrain_loads_msg.force[i][0];
    m_terrain_loads_array[i](1) = terrain_loads_msg.force[i][1];
    m_terrain_loads_array[i](2) = terrain_loads_msg.force[i][2];

    m_terrain_loads_array[i](3) = terrain_loads_msg.moment[i][0];
    m_terrain_loads_array[i](4) = terrain_loads_msg.moment[i][1];
    m_terrain_loads_array[i](5) = terrain_loads_msg.moment[i][2];
  }
}

void FmuComponent::logData(void) {
  if (GetTime() >= m_next_log_time) {
    logTagged("logData: " + stateLine(), fmi3Status::fmi3OK, "logAll");
    m_next_log_time += params.log_interval;
  }
}

// Build a neat one-line summary of all four m_tires' state + loads.
std::string FmuComponent::stateLine() {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4);
  ss << "t=" << GetTime();
  for (int i = 0; i < m_PADS; i++) {
    ss << "\npad " << i << "\n "
       << "\tpos=(" << m_states_array[i](0) << ", " << m_states_array[i](1)
       << ", " << m_states_array[i](2) << ")"
       << "\trot=(" << m_states_array[i](3) << ", " << m_states_array[i](4)
       << ", " << m_states_array[i](5) << ", " << m_states_array[i](6) << ")"
       << "\n"
       << "\tlinvel=(" << m_states_array[i](7) << ", " << m_states_array[i](8)
       << ", " << m_states_array[i](9) << ")"
       << "\tangvel=(" << m_states_array[i](10) << ", " << m_states_array[i](11)
       << ", " << m_states_array[i](12) << ")" << "\n"
       << "\tFx=" << m_terrain_loads_array[i](0)
       << "\tFy=" << m_terrain_loads_array[i](1)
       << "\tFz=" << m_terrain_loads_array[i](2) << "\n"
       << "\tMx=" << m_terrain_loads_array[i](3)
       << "\tMy=" << m_terrain_loads_array[i](4)
       << "\tMz=" << m_terrain_loads_array[i](5);
  }
  return ss.str();
}

void FmuComponent::TerminateExternalProcess() {
  // Idempotent: if we've already torn down, do nothing.
  if (m_terminated)
    return;
  m_terminated = true;

  // 1. Close the socket and context so the external process's blocking
  //    recv() errors/times out and it can self-exit.
  try {
    sock.close();
  } catch (...) {
    // ignore -- socket may already be closed
  }
  try {
    ctx.shutdown(); // unblocks any pending zmq calls
    ctx.close();
  } catch (...) {
    // ignore
  }

  // 2. Give the external process a chance to exit cleanly, then force it.
  if (m_procInfo.hProcess != nullptr) {
    DWORD wait_result = WaitForSingleObject(m_procInfo.hProcess, 5000);
    if (wait_result != WAIT_OBJECT_0) {
      // still running after 5 s -> force terminate
      TerminateProcess(m_procInfo.hProcess, 0);
      // give the forced kill a brief moment to take effect
      WaitForSingleObject(m_procInfo.hProcess, 1000);
    }

    CloseHandle(m_procInfo.hProcess);
    m_procInfo.hProcess = nullptr;
  }

  if (m_procInfo.hThread != nullptr) {
    CloseHandle(m_procInfo.hThread);
    m_procInfo.hThread = nullptr;
  }

  logTagged("terminateExternalProcess: Terminated external process!",
            fmi3Status::fmi3OK, "logAll");
}