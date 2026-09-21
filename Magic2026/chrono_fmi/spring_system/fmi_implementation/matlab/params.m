% add fmu to path
addpath("C:\Users\ahmed\Documents\sbel\magic_2026\tutorials\spring_system\fmi_implementation\build\FMU3cs_two_mass_spring")

% simulation parameters
timestep = 1e-4;

%% body parameters
% chrono body
size_chrono = 0.1;
dim_chrono = size_chrono * ones(1, 3);
mass_chrono = 1.5;

% simscape body
size_simscape = 0.1;
dim_simscape = size_simscape * ones(1, 3);
mass_simscape = 2;

% spring
spring_rest_length = 0.1;
spring_stiffness = 500;

%% initial conditions
init_pos_chrono = 0.2;
init_pos_simscape = 0.42;

%% force
force_amplitude = 2;
force_freq = 1 / 5;

