// =============================================================================
// Authors: Ahmed Ansari
// =============================================================================
// Simple example of two boxes free to move along x axis, connected by a
// spring and an applied force on one box
// all units are SI
//
// Chrono body and spring live inside chrono FMU; output: spring force;
// input:Simscape body pos
//
// Simscape body and applied force live inside Simscape;
// output: simscape body pos; intput: spring force

#include "two_mass_spring_fmu.h"

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

  // Set start values for FMU input and output variables
  // other values are set in the header file
  x_s = 0.40;
  F_sp = 0;

  // register the unit for stiffness
  // Register the N/m unit (kg·s^-2), not in the common set
  addUnitDefinition(fmu_forge::UnitDefinition("N/m", 1, 0, -2, 0, 0, 0, 0, 0));

  // Set FMU paramters
  AddFmuVariable(&size_chrono, "size_chrono_body", FmuVariable::Type::Float64,
                 "m", "chrono body dimension",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed);
  AddFmuVariable(&mass_chrono, "mass_chrono_body", FmuVariable::Type::Float64,
                 "kg", "chrono body mass",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed);
  AddFmuVariable(&spring_rest_length, "spring_rest_length",
                 FmuVariable::Type::Float64, "m", "spring rest length",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed);
  AddFmuVariable(&spring_stiffness, "spring_stiffness",
                 FmuVariable::Type::Float64, "N/m", "spring stiffness",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed);
  AddFmuVariable(
      &init_xpos_chrono, "init_xpos_chrono", FmuVariable::Type::Float64, "m",
      "chrono body initial x position", FmuVariable::CausalityType::parameter,
      FmuVariable::VariabilityType::fixed);
  AddFmuVariable(&init_xpos_simscape, "init_xpos_simscape",
                 FmuVariable::Type::Float64, "m",
                 "simscape body initial x position",
                 FmuVariable::CausalityType::parameter,
                 FmuVariable::VariabilityType::fixed);

  // Set FMU input
  AddFmuVariable(&x_s, "xpos_simscape", FmuVariable::Type::Float64, "m",
                 "simscape body x position", FmuVariable::CausalityType::input,
                 FmuVariable::VariabilityType::continuous);

  // Set FMU output
  AddFmuVariable(&F_sp, "spring_force", FmuVariable::Type::Float64, "N",
                 "spring force", FmuVariable::CausalityType::output,
                 FmuVariable::VariabilityType::continuous);

  //  for logging only
  AddFmuVariable(&x_c, "xpos_chrono", FmuVariable::Type::Float64, "m",
                 "chrono body x position", FmuVariable::CausalityType::output,
                 FmuVariable::VariabilityType::continuous);

  // specify variable dependencies
  DeclareVariableDependencies("spring_force",
                              {"xpos_simscape", "size_chrono_body",
                               "mass_chrono_body", "spring_rest_length",
                               "spring_stiffness", "init_xpos_chrono"});

  DeclareVariableDependencies("xpos_chrono",
                              {"size_chrono_body", "mass_chrono_body",
                               "spring_rest_length", "spring_stiffness",
                               "init_xpos_chrono"});

  // Specify functions to process input variables (at beginning of step)
  AddPreStepFunction([this]() { this->SetDummyBodyPos(); });

  // Specify functions to calculate FMU outputs (at end of step)
  AddPostStepFunction([this]() { this->CalculateOutputs(); });
}

// definition for processing dummy body pos
void FmuComponent::SetDummyBodyPos() {
  dummy_body_simscape->SetPos(ChVector3d(x_s, 0, 0));
}

// defintion for calculating output spring force
void FmuComponent::CalculateOutputs() {
  F_sp = spring->GetForce();
  x_c = body_chrono->GetPos().x();

  if (GetTime() >= next_log_time) {
    logData(
        "t = " + std::to_string(m_time) + ",\tx_c = " + std::to_string(x_c) +
        ",\tx_s = " + std::to_string(x_s) + ",\tF_s = " + std::to_string(F_sp));

    next_log_time += log_interval;
  }
}

// this is run before creating the model description at build time
void FmuComponent::preModelDescriptionExport() {}

// this is run after creating the model description at build time
void FmuComponent::postModelDescriptionExport() {}

// this is run before receiving parameters from the master
fmi3Status FmuComponent::enterInitializationModeIMPL() {
  return fmi3Status::fmi3OK;
}

// this is run after receiving the parameters from master
fmi3Status FmuComponent::exitInitializationModeIMPL() {

  sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
  m_stepSize = h; // FMU internal timestep

  sys.SetGravityZ(); // sets 9.81 in the -Z automatically

  // add ground
  ground = chrono_types::make_shared<ChBody>();
  ground->SetFixed(true);
  ground->SetPos(ChVector3d(0, 0, 0)); // not required explicitly
  sys.AddBody(ground);

  // create body chrono and give it geometric and inertial parameters
  body_chrono = chrono_types::make_shared<ChBodyEasyBox>(
      size_chrono, size_chrono, size_chrono, 1000, true, false);
  sys.AddBody(body_chrono);

  ChVector3d body_chrono_init_pos(init_xpos_chrono, 0.0, 0.0);

  body_chrono->SetName("body_chrono");
  body_chrono->SetPos(body_chrono_init_pos);
  body_chrono->SetLinVel(VNULL); // velocity need not be set to 0 explicitly
  body_chrono->SetMass(mass_chrono);

  // create body simscape and give it geometric and inertial parameters
  dummy_body_simscape = chrono_types::make_shared<ChBody>();
  sys.AddBody(dummy_body_simscape);

  dummy_body_simscape->SetName("body_simscape");
  dummy_body_simscape->SetPos(ChVector3d(init_xpos_simscape, 0, 0));
  dummy_body_simscape->SetFixed(true); // SetDummyBodyPos() sets the position

  // create prismatic joints with the world with zero friction
  auto joint_prismatic_1 = chrono_types::make_shared<ChLinkMatePrismatic>();
  joint_prismatic_1->Initialize(
      ground, body_chrono,
      ChFrame<>(ground->GetPos(), QuatFromAngleY(CH_PI_2)));
  sys.AddLink(joint_prismatic_1);

  auto joint_prismatic_2 = chrono_types::make_shared<ChLinkMatePrismatic>();
  joint_prismatic_2->Initialize(
      ground, dummy_body_simscape,
      ChFrame<>(ground->GetPos(), QuatFromAngleY(CH_PI_2)));
  sys.AddLink(joint_prismatic_2);

  // connect spring between the masses
  spring = chrono_types::make_shared<ChLinkTSDA>();
  spring->Initialize(body_chrono, dummy_body_simscape, true,
                     ChVector3d(size_chrono / 2, 0, 0),
                     ChVector3d(-dummy_size_simscape / 2, 0, 0));
  spring->SetRestLength(spring_rest_length);
  spring->SetSpringCoefficient(spring_stiffness);
  sys.AddLink(spring);

  sys.DoAssembly(AssemblyAnalysis::Level::FULL);

  CalculateOutputs();

  sendToLog("Initialization done!", fmi3Status::fmi3OK, "logAll");

  return fmi3Status::fmi3OK;
}

void FmuComponent::logData(std::string msg) {
  sendToLog(msg, fmi3Status::fmi3OK, "logAll");
}

// for running the FMU dynamics
fmi3Status FmuComponent::doStepIMPL(
    fmi3Float64 currentCommunicationPoint, fmi3Float64 communicationStepSize,
    fmi3Boolean noSetFMUStatePriorToCurrentPoint,
    fmi3Boolean *eventHandlingNeeded, fmi3Boolean *terminateSimulation,
    fmi3Boolean *earlyReturn, fmi3Float64 *lastSuccessfulTime) {
  // Advance FMU state to next communication point
  while (m_time < currentCommunicationPoint + communicationStepSize) {
    fmi3Float64 step_size =
        std::min((currentCommunicationPoint + communicationStepSize - m_time),
                 std::min(communicationStepSize, m_stepSize));

    sys.DoStepDynamics(step_size);

    m_time += step_size;
  }

  return fmi3Status::fmi3OK;
}
