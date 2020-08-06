% Segway Comparison
clear;
clc;
close all;
%% Load data
clear;
LMPC_data = load('saved_results/u_LMPC_dynamics_nonlinear.mat');
t_LMPC = LMPC_data.t_sim;
x_LMPC = LMPC_data.x_sim;
params_LMPC = LMPC_data.params;

Linear_data = load('saved_results/u_Linear_dynamics_nonlinear.mat');
t_Linear = Linear_data.tout;
x_Linear = Linear_data.xout;
params_Linear = Linear_data.params;

%% Recompute torques
uout_LMPC = Extract_Torques(t_LMPC,x_LMPC,params_LMPC);
uout_Linear = Extract_Torques(t_Linear,x_Linear,params_Linear);

%% Comparison plot
figure
q_header = ["\phi","\theta","d \phi","d \theta"];
for i = 1:4
    subplot(2,2,i)
    plot(t_Linear,x_Linear(:,i)); hold on;
    plot(t_LMPC,x_LMPC(:,i));
    title(q_header(i));
end
legend('Linear','LMPC');
    
figure
plot(t_Linear,uout_Linear); hold on;
plot(t_LMPC,uout_LMPC);
legend('Linear', 'LMPC');
title('Motor Torques [u]');
   











