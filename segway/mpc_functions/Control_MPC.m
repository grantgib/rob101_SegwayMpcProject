function [u_mpc] = Control_MPC(A, B)
%CONTROL_MPC - This function computes the Model Predictive Control input
%given a linear plant model

%% Extract Input Data
dyn_struct = struct('A_c',  A,...
                    'B_c',  B);
Ts = 0.1;

%% Discretize Linear Dynamics
[A_d, B_d] = Discretize_Dynamics(dyn_struct,Ts);

%% Formulate matrices for Quadratic Program Solver
[H, L, G, W, T, IMPC] = Form_QP_Matrices(A_d, B_d, Q, R, P, xlim, ulim, N, slackPenalty);

%% Solve QP - control input sequence solution
[U, lam] = Solve_QP(H, L, G, W, T, IMPC);

%% Apply first control input in the solution sequence
u_mpc = U(:,1);

end

