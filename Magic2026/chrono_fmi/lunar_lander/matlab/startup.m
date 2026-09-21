clc;
clear;

fmu_path = ".\..\chrono_fmu\fmu\build\FMU3cs_crm";
addpath(fmu_path);

% open simulink/simscape model
lander_simscape

lander_z_init = 3; % m
lander_pitch_init = 15; % deg

camera_pos = [-20, -10, 3];