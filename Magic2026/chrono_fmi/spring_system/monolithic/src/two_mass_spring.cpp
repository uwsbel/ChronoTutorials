// imple example of two boxes free to move along x axis, connected a by a
// spring and a force on one box
// all units are SI

#include "chrono/functions/ChFunctionSine.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChSystemSMC.h"
#include <chrono/core/ChRealtimeStep.h>
#include <fstream>
#include <iostream>

// Use the namespaces of Chrono
using namespace chrono;

// sim parameters
double h = 1e-4; // timestep
double t_end = 10.0;

// parameters
ChVector3d body_chrono_init_pos(0.2, 0.0, 0.0),
    body_simscape_init_pos(0.42, 0.0, 0.0);

double spring_rest_length = 0.10, spring_stiffness = 500;

double size_chrono = 0.1, size_simscape = 0.1;
double mass_chrono = 1.5, mass_simscape = 2;

double force_amplitude = 2.0, force_freq = 1.0 / 5.0;

int main(int argc, char *argv[]) {

  ChSystemSMC sys;

  sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

  sys.SetGravityZ(); // sets 9.81 in the -Z automatically

  // add ground
  auto ground = chrono_types::make_shared<ChBody>();
  ground->SetFixed(true);
  ground->SetPos(ChVector3d(0, 0, 0)); // not required explicitly
  sys.AddBody(ground);

  // create body chrono and give it geometric and inertial parameters
  auto body_chrono = chrono_types::make_shared<ChBodyEasyBox>(
      size_chrono, size_chrono, size_chrono, 1000, true, false);
  sys.AddBody(body_chrono);

  body_chrono->SetName("body_chrono");
  body_chrono->SetPos(body_chrono_init_pos);
  body_chrono->SetLinVel(VNULL); // velocity need not be set to 0 explicitly
  body_chrono->SetMass(mass_chrono);

  // create body simscape and give it geometric and inertial parameters
  auto body_simscape = chrono_types::make_shared<ChBodyEasyBox>(
      size_simscape, size_simscape, size_simscape, 1000, true, false);
  sys.AddBody(body_simscape);

  body_simscape->SetName("body_simscape");
  body_simscape->SetPos(body_simscape_init_pos);
  body_simscape->SetLinVel(VNULL); // velocity need not be set to 0 explicitly
  body_simscape->SetMass(mass_simscape);

  // create prismatic joints with the world with zero friction
  auto joint_prismatic_1 = chrono_types::make_shared<ChLinkMatePrismatic>();
  joint_prismatic_1->Initialize(
      ground, body_chrono,
      ChFrame<>(ground->GetPos(), QuatFromAngleY(CH_PI_2)));
  sys.AddLink(joint_prismatic_1);

  auto joint_prismatic_2 = chrono_types::make_shared<ChLinkMatePrismatic>();
  joint_prismatic_2->Initialize(
      ground, body_simscape,
      ChFrame<>(ground->GetPos(), QuatFromAngleY(CH_PI_2)));
  sys.AddLink(joint_prismatic_2);

  // create spring between the masses
  auto spring = chrono_types::make_shared<ChLinkTSDA>();
  spring->Initialize(body_chrono, body_simscape, true,
                     ChVector3d(size_chrono / 2, 0, 0),
                     ChVector3d(-size_simscape / 2, 0, 0));
  spring->SetRestLength(spring_rest_length);
  spring->SetSpringCoefficient(spring_stiffness);
  sys.AddLink(spring);

  // create force actuator connected to body simscape and give it a force
  // profile of 2*sin(2*pi/5*t)
  auto force = chrono_types::make_shared<ChForce>();
  body_simscape->AddForce(force); // FIRST, as docs require
  force->SetMode(ChForce::ForceType::FORCE);
  force->SetDir(ChVector3d(1, 0, 0));
  force->SetMforce(force_amplitude); // <-- the missing magnitude (2 N base)
  auto f_t = chrono_types::make_shared<ChFunctionSine>(
      1.0, force_freq);      // modulation in [-1,1]
  force->SetModulation(f_t); // multiplies Mforce -> 2*sin(...)

  std::ofstream csv(std::string(PROJECT_RESULTS_DIR) + "/two_mass_spring.csv");
  csv << "t,applied_force,x_chrono,x_simscape,spring_force\n";

  csv << sys.GetChTime() << "," << force->GetForce().x() << ","
      << body_chrono->GetPos().x() << "," << body_simscape->GetPos().x() << ","
      << spring->GetForce() << "\n";

  while (sys.GetChTime() < t_end) {
    sys.DoStepDynamics(h);
    csv << sys.GetChTime() << "," << force->GetForce().x() << ","
        << body_chrono->GetPos().x() << "," << body_simscape->GetPos().x()
        << "," << spring->GetForce() << "\n";
  }
  csv.close();
  std::cout << "Done. Wrote two_mass_spring.csv\n";

  return 0;
}