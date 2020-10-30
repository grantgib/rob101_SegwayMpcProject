%% Segway_main - Top level script for running the Segway code
%   In the file Segway_simulate, you can select to simulate a 
%   nonlinear model or a linear model
%   
%   x = state = [phi; theta; dphi/dt; dtheta/dt]
%
%       phi         = shaft angle position (lean/steer frame angle)
%       theta       = wheel angle position
%       dphi/dt     = shaft angle velocity
%       dtheta/dt   = wheel angle velocity

%% Add necessary directories to path
clc; clear; close all;
addpath(genpath('segway_functions'));
addpath(genpath('mpc_functions'));

%% Set Initial Conditon & Simulate
segprob = "vel";    % "pos" or "vel"
t_end = 50;       % Time to simulate 
[t_sim,x_sim,params_sim] = Segway_simulate(t_end,segprob);

%% Plot State/Output variables
Segway_plot(t_sim,x_sim,params_sim);

%% Animation
% animation_speed = 1;
% Segway_anim(t_sim,x_sim(:,1),x_sim(:,2),animation_speed);















