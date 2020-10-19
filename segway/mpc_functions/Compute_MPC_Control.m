function [u_mpc] = Compute_MPC_Control(params)
%CONTROL_MPC - This function computes the Model Predictive Control input
%given a linear plant model

%% Initialize variables
x0 = params.x0;
H = params.H;
Aineq = params.G;
L = params.L;
W = params.W;
T = params.T;
IMPC = params.IMPC;
solver_type = "my_solver";  % my_solver, matlab_solver

%% Solve QP - control input sequence solution
% Minimize 1/2*U^T H U + q^{\rm T} U subject to G U <= W +T*x_0 = Wtilde
%   where q = L*x_0
if solver_type == "my_solver"
    %% Use Dual projected gradient algorithm for QP solver
    q = L*x0;                   % Linear cost vector
    Bineq = W + T*x0;          % Inequality bound matrix
    [U, lambda] = Solve_QP_DualProjectedGradient(H,q,Aineq,Bineq,[],[],[],[]);
    
elseif solver_type == "matlab_solver"
    %% QUADPROG Solution (Built-in Matlab solver)
    q = L*x0;
    Bineq = W + T*x0;
    options = optimoptions(@quadprog,'Display','off');
    [U,~,EXITFLAG] = quadprog(H,q,Aineq,Bineq,[],[],[],[],[],options);
    if EXITFLAG ~= 1
        disp('Invalid solution for some reason');
        pause;
    end
end

%% Apply first control input in the solution sequence
u_mpc = IMPC*U;

end

