#pragma once
#include <cstdint>

constexpr int m_PADS = 4;

// Sent ONCE at startup: the parameters
#pragma pack(push,                                                             \
             1) // pack tightly so both sides agree on layout; prevents compiler
                // specific padding; no-op in our case since same compiler

// Sent each step: the inputs
struct StateMsg {
  double pos[m_PADS][3];
  double rot[m_PADS][4];
  double lin_vel[m_PADS][3];
  double ang_vel[m_PADS][3];
  double current_time;
  double communication_step_size;
  double step_size;
  bool save_results_to_file_flag;
};

// Returned each step: the output
struct TerrainLoadMsg {
  double force[m_PADS][3];
  double moment[m_PADS][3];
  int8_t save_results_to_file_ack;
};

// all parameters
struct ParamsMsg {
 
  // sim and log parameters
  double h;
  double h_cfd;        // for cfd
  double log_interval; // StateMsg init_tire_config;

  //   terrain params
  // floor geometry
  double patch_length; // x:room fore/aft
  double patch_width;  // y
  double patch_depth;  // z

  // CRM (moving) active box dimension
  bool enable_crm_active_domain;
  double active_box_dim;
  double active_domain_settling_time;

  // CRM material properties
  // some material properties and solver parameters are hardcoded inside the
  // terrain program
  double density;
  double cohesion;
  double friction;
  double youngs_modulus;
  double poisson_ratio;

  int num_proximity_search_steps;

  // terrain spacing
  double terrain_spacing;

  // soil csv save path for post processing; sent and recvd as a char array;
  // cannot use string since not a POD

  //   exe write
  bool exe_out_to_file;

  //   path params
  char resource_path[1024];
  char terrain_save_path[1024];
  char foot_pad_mesh_file_path[1024];
  char exe_stdout_path[1024];
  char exe_stderr_path[1024];

  // vis params
  bool vis_flag; // visualization control flag for the fmu
  bool capture_frames;

  // vis params
  double vis_camera_pos[3];
  bool vis_camera_chase_target;
  bool body_vis;
  bool mesh_vis;
  double vis_render_fps;
};

#pragma pack(pop)