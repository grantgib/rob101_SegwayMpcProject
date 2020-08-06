%% Segway_main - Top level script for running the Segway code
%   In the file Segway_simulate, you can select to simulate a 
%   nonlinear model or a linear model
%   
%   x = state = [phi, theta, dphi/dt, dtheta/dt]
%       phi         = shaft angle position (lean/steer frame angle)
%       theta       = wheel angle position
%       dphi/dt     = shaft angle velocity
%       dtheta/dt   = wheel angle velocity


clc; clear; close all;

%% Add necessary directories to path
addpath(genpath('segway_functions'));
addpath(genpath('mpc_functions'));

%% Set Initial Conditon & Simulate
x0 = [0;10;0;0] ; % Initial condition
t_end = 25;       % Time to simulate 
[t_sol, x_sol] = Segway_simulate(t_end,x0);

%% Plot State/Output variables
Segway_plot(t_sol,x_sol);

%% Animation
animation_speed = 1;
Segway_anim(t_sol,x_sol(:,1),x_sol(:,2),animation_speed);

















