function [dxdt,u] = Segway_ode(t,x,params)
%% SEGWAY_ODE - Compute ODE for segway model
%   This function takes in the current time, state, and params to
%   calculate the differential equation for the segway. 

%% Extract params data
sim_type = params.sim_type;
params.x0 = x;

%% Simulate using linear/nonlinear model and linear/nonlinear controller
if sim_type == "LINEAR"
    % Simulate Linear dynamics
    A = [0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
    B = [0;0;-0.7172;1.6744];
    u = Compute_MPC_Control(params);
    dxdt = A*x + B*u;
else
    % Simulate Nonlinear dynamics
    q = x(1:2);
    dq = x(3:4);
    [D_dyn,C_dyn,G_dyn,B_dyn]= dyn_mod_segway(q,dq);
    A = [0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
    B = [0;0;-0.7172;1.6744];
    u = Compute_MPC_Control(params);
    dxdt = [x(3:4);
            D_dyn\(-C_dyn*dq - G_dyn + B_dyn*u)];
end

end

