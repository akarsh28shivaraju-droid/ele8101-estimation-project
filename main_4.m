clear; clc; close all;
addpath('src/step4');

C = config_4();
plot_track_geometry(C);
run_ekf_4(C);
