function [u_mpc] = Compute_MPC_Control(params)
%CONTROL_MPC - This function computes the Model Predictive Control input
%given a linear plant model

%% Initialize variables
dim_u = params.dim_u;
solver_type = "matlab_solver";  % my_solver, matlab_solver

%% Compute QP Matrices
[H,q,Ain,bin] = Compute_QP_Matrices(params);

%% Solve QP - control input sequence solution
% Minimize 1/2*U'*H*U + q'*U subject to Ain*U <= bin
if solver_type == "my_solver"
    %% Use Dual projected gradient algorithm for QP solver
    [U, lambda] = Solve_QP_DualProjectedGradient(H,q,Ain,bin,[],[],[],[]);
    
elseif solver_type == "matlab_solver"
    %% QUADPROG Solution (Built-in Matlab solver)
    options = optimoptions(@quadprog,'Display','off');
    [U,~,EXITFLAG] = quadprog(H,q,Ain,bin,[],[],[],[],[],options);
    if EXITFLAG ~= 1
        disp('Invalid solution for some reason');
        error('Quadprog could not find solution');
    end
end

%% Apply first control input in the solution sequence
u_mpc = U(1:dim_u);

end

%% 
function [H,q,Ain,bin] = Compute_QP_Matrices(params)
% Extract params
N = params.N;
X_goal = params.X_goal;
X_max = params.X_max;
X_min = params.X_min;
U_max = params.U_max;
U_min = params.U_min;
S = params.S;
M = params.M;
Qstate_bar = params.Qstate_bar;
Qctrl_bar = params.Qctrl_bar;
x_init = params.x_init;
dim_u = params.dim_u;

% compute QP matrices
H = S'*Qstate_bar*S + Qctrl_bar;
q = (2*(M*x_init - X_goal)'*Qstate_bar*S)';
Ain = [S;
       -S;
       eye(N*dim_u);
       -eye(N*dim_u)];
bin = [X_max - M*x_init;
       -X_min + M*x_init;
       U_max;
       -U_min];
end

