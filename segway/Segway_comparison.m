% Segway Comparison
%% Load data
clear;
LMPC_data = load('saved_results/u_LMPC_dynamics_nonlinear.mat');
tout_LMPC = LMPC_data.tout;
xout_LMPC = LMPC_data.xout;
params_LMPC = LMPC_data.params;

Linear_data = load('saved_results/u_Linear_dynamics_nonlinear.mat');
tout_Linear = Linear_data.tout;
xout_Linear= Linear_data.xout;
params_Linear = Linear_data.params;

%% Recompute torques
uout_LMPC = Extract_Torques(tout_LMPC,xout_LMPC,params_LMPC);
uout_Linear = Extract_Torques(tout_Linear,xout_Linear,params_Linear);

%% Comparison plot
figure
q_header = ["\phi","\theta","d \phi","d \theta"];
for i = 1:4
    subplot(2,2,i)
    plot(tout_Linear,xout_Linear(:,i)); hold on;
    plot(tout_LMPC,xout_LMPC(:,i));
    title(q_header(i));
end
legend('Linear','LMPC');
    
figure
plot(tout_Linear,uout_Linear); hold on;
plot(tout_LMPC,uout_LMPC);
legend('Linear', 'LMPC');
title('Motor Torques [u]');
   











