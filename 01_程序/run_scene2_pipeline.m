%RUN_SCENE2_PIPELINE Run the full scene2 RRRP kinematics pipeline.

clear; clc; close all;

codeDir = fileparts(mfilename('fullpath'));
addpath(codeDir);

run_kinematics_solution;
run_trajectory_planning;
run_joint_pd_tracking_simulation;
run_joint_pd_tracking_animation;
run_rrrp_kinematics_animation;
run_scene2_urdf_kinematics_visualization;
