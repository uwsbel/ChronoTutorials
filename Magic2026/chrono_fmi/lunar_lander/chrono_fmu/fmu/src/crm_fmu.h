// =============================================================================
// Authors: Ahmed Ansari
// =============================================================================

#pragma once

#include "messages.h"
#include <Windows.h>
#include <cstring>
#include <string>
#include <zmq.hpp>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChSpindle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

// required for FMI
#include "chrono_fmi/fmi3/ChFmuToolsExport.h"

class FmuComponent : public chrono::fmi3::FmuChronoComponentBase {
public:
  FmuComponent(fmu_forge::fmi3::FmuType fmiInterfaceType,
               fmi3String instanceName, fmi3String instantiationToken,
               fmi3String resourcePath, fmi3Boolean visible,
               fmi3Boolean loggingOn,
               fmi3InstanceEnvironment instanceEnvironment,
               fmi3LogMessageCallback logMessage);

  virtual ~FmuComponent(); // destructor

  /// Advance dynamics
  virtual fmi3Status doStepIMPL(fmi3Float64 currentCommunicationPoint,
                                fmi3Float64 communicationStepSize,
                                fmi3Boolean noSetFMUStatePriorToCurrentPoint,
                                fmi3Boolean *eventHandlingNeeded,
                                fmi3Boolean *terminateSimulation,
                                fmi3Boolean *earlyReturn,
                                fmi3Float64 *lastSuccessfulTime) override;

protected:
  virtual fmi3Status enterInitializationModeIMPL() override;
  virtual fmi3Status exitInitializationModeIMPL() override;

  virtual void preModelDescriptionExport() override;
  virtual void postModelDescriptionExport() override;

  virtual bool is_cosimulation_available() const override { return true; }
  virtual bool is_modelexchange_available() const override { return false; }

  // Custom system specific code
  void ProcessInputs();
  void ProcessOutputs();

  void logData(void);
  std::string stateLine(void);

  void ChronoStates2POD();
  void POD2ChronoTerrainLoads();

  bool LaunchSolver(const std::string &);

  void logTagged(std::string, fmi3Status, std::string);

  void TerminateExternalProcess();

  bool m_terminated = false;

  static constexpr int m_STATE_SIZE = 13;
  static constexpr int m_TERRAIN_LOAD_SIZE = 6;

  // params, inputs and outputs
  ParamsMsg params;
  StateMsg states_msg;
  TerrainLoadMsg terrain_loads_msg;

  double m_sim_end_time;

  // received from chrono
  std::string m_terrain_save_path;
  bool m_save_terrain_flag;

  std::string m_foot_pad_mesh_file_path;

  // message
  zmq::context_t ctx;
  zmq::socket_t sock;

  // process info
  PROCESS_INFORMATION m_procInfo;

  // working directory is the resources folder
  // of the unpacked fmu (created by Simscape);
  // exe is the complete filepath of the crm executable
  std::string exe_path, work_dir;

  double m_current_time, m_next_log_time;

  // inputs
  std::array<Eigen::VectorXd, m_PADS> m_states_array;

  // outputs
  std::array<Eigen::VectorXd, m_PADS> m_terrain_loads_array;

  // tire ChBody declaration
  std::array<std::shared_ptr<chrono::ChBody>, m_PADS> m_pads;

  // exe stdout path
  std::string m_exe_stdout_path, m_exe_stderr_path;

  // vis params
  chrono::ChVector3d m_vis_camera_pos;
};
