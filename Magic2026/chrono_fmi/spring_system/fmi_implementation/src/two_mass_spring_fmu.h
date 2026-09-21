// =============================================================================
// Authors: Ahmed Ansari
// =============================================================================

#pragma once

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChSystemSMC.h"

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
  virtual ~FmuComponent() {}

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
  void SetDummyBodyPos();
  void CalculateOutputs();

  void logData(std::string);

  // Chrono sys object
  chrono::ChSystemSMC sys;

  // sim parameters
  double h = 1e-4; // timestep

  // Body parameters (with default values)
  double spring_rest_length = 0.10, spring_stiffness = 500;

  double size_chrono = 0.1; // cube
  double mass_chrono = 1.5;

  double dummy_mass_simscape = 2.0; // not used
  double dummy_size_simscape = 0.1;

  double init_xpos_chrono = 0.2;
  double init_xpos_simscape;

  // I/O variables
  double x_s;
  double F_sp;

  double x_c; // for logging/debugging only

  // Body definitions
  std::shared_ptr<chrono::ChBody> ground;
  std::shared_ptr<chrono::ChBodyEasyBox> body_chrono;
  std::shared_ptr<chrono::ChBody> dummy_body_simscape;

  // TSDA spring
  std::shared_ptr<chrono::ChLinkTSDA> spring;

  // logging time variables
  double log_interval = 0.1, next_log_time = 0;
};
