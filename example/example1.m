% Example 1
clear; clc;
x0 = [0;2];
u0 = 3;
dt = 0.1;
N = 3;
[X_traj,U_traj] = Discrete_Dynamics_Update(x0,u0,dt,N)
function [X_traj,U_traj] = Discrete_Dynamics_Update(x_init,u_init,delta_T,N)
A = [1, delta_T; 0 1];
B = [0; delta_T];
U_traj = zeros(length(u_init),N);
X_traj = zeros(length(x_init),N+1);
for i = 1:N+1
    if i == 1
        U_traj(:,i) = u_init;
        X_traj(:,i) = x_init;
    else
        U_traj(:,i) = u_init;
        X_traj(:,i) = A*X_traj(:,i-1) + B*U_traj(:,i-1);
    end
end
end