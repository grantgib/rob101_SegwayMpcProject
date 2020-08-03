function [u_mpc] = Compute_MPC_Control(A,B)
%CONTROL_MPC - This function computes the Model Predictive Control input
%given a linear plant model

%% Extract Input Data
dyn_struct = struct('A_c',  A,...
                    'B_c',  B);
Ts = 0.1;

%% Discretize Linear Dynamics
[A_d, B_d] = Discretize_Dynamics(dyn_struct,Ts);

%% Formulate matrices for Quadratic Program Solver
[H, L, G, W, T, IMPC] = Form_QP_Matrices(A_d, B_d, Q, R, P, xlim, ulim, N);

%% Solve QP - control input sequence solution
% Minimize 1/2*U^T H U + q^{\rm T} U subject to G U <= W +T*x_0 = Wtilde
% where q = L*x_0
[U, lam] = Solve_QP(H,q,G,Wtilde,lam0,maxIter);

%% Apply first control input in the solution sequence
u_mpc = U(:,1);

end

