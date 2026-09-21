#include "messages.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <zmq.hpp>

#include "chrono/geometry/ChTriangleMeshConnected.h"
// #include "chrono/input_output/ChWriterCSV.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono/collision/bullet/ChCollisionSystemBullet.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChSpindle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono/input_output/ChWriterCSV.h"

#include "chrono/utils/ChUtils.h"

#include "SetChronoSolver.h"

#include <algorithm>

#include <chrono>
#include <fstream>

#include <filesystem>

void AssignStates(std::shared_ptr<chrono::ChBody> &, int, const StateMsg &);

void GetTerrainLoads(TerrainLoadMsg &, const std::shared_ptr<chrono::ChBody> &,
                     int, const chrono::vehicle::CRMTerrain &);

int CheckQuatNorm(chrono::ChQuaterniond &);

std::string stateLine(double, const ParamsMsg &, const StateMsg &,
                      const TerrainLoadMsg &);