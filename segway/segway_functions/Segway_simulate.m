function [t_sim,x_sim,params] = Segway_simulate(t_end,segprob)
%% Segway_simulate - Generate state trajectory for segway given initial condition

%% Initial Conditions
if segprob == "pos"
    x_init = [0;10;0;0] ; % Initial condition
    x_goal = [0;0;0;0];
else
    x_init = [0;0;0;0];
    x_goal = [0;0;0;1];
end

%% Linear Dynamics Differential Equation
%   dx/dt = A*x + B*u
A = [0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
B = [0;0;-0.7172;1.6744];
dyn_struct = struct('A_c',A,'B_c',B);   % Create struct for continuos-time matrices

%% Simulation settings 
% ctrl_type:    choose which controller to use (LMPC)
% sim_type:     choose which dynamics to forward integrate (LINEAR, NONLINEAR)
ctrl_type = "LMPC";
sim_type = "LINEAR";

%% Discretize Linear Dynamics
Ts = 0.1;   % Time discretization (units in seconds)
[A_d, B_d] = Discretize_Dynamics(dyn_struct,Ts); % Discrete-time matrices

%% Formulate matrices for Quadratic Program Solver
N = 5;                              % Prediction Horizon
if segprob == "pos"
    Qctrl = 0.01; % Input penalty matrix (only one input so 1x1)
    Qstate = diag([1 1 100 1]);                 % State penalty diagonal matrix                          
    [~,Q_term] = dlqr(A_d,B_d,Qstate,Qctrl);    % Terminal state penalty                
else
    Qctrl = 0.01;
    Qstate = diag([0.0001 0.0001 1 1e4]);
    [~,Q_term] = dlqr(A_d,B_d,Qstate,Qctrl);
%     Qstate = diag([0 0 1 1000]);
%     Q_term = 1e3*Qstate;
end

xlim = struct('max',    [inf;inf;inf;inf],...
              'min',    -[inf;inf;inf;inf]);    % state limits
ulim = struct('max',    3,...
              'min',    -3);                  % input/control limits

% Minimize 1/2*U'*H*U + q*U subject to Ain*U <= bin
%   IMPC extracts the first control move, u_MPC = IMPC*U

[S,M] = Compute_State_Transition_Matrices(A_d, B_d, N);
[Qstate_bar,Qctrl_bar] = Compute_Penalty_Matrices(Qstate,Q_term,Qctrl,N);
[X_goal,X_max,X_min,U_max,U_min] = ...
    Compute_Stacked_Constraints_Goals_Matrices(x_goal,xlim.max,xlim.min,ulim.max,ulim.min,N);

% [H, L, G, W, T, IMPC] = Form_QP_Matrices(A_d, B_d, Q, R, Q_term, xlim, ulim, N);
%% Forward Integration Settings
options = odeset('Refine',4,'RelTol',1e-7,'AbsTol',1e-8);
t_start = 0;
params = struct('sim_type',     sim_type,...
                'ctrl_type',    ctrl_type,...
                'N',            N,...
                'S',            S,...
                'M',            M,...
                'Qstate_bar',   Qstate_bar,...
                'Qctrl_bar',    Qctrl_bar,...
                'X_goal',       X_goal,...
                'X_min',        X_min,...
                'X_max',        X_max,...
                'U_min',        U_min,...
                'U_max',        U_max,...
                'dim_u',        size(B,2));
            
%% Simulate with desired Forward Integration Scheme(Runge Kutta Fehlberg 45)
[t_sim, x_sim] = ode45(@(t,x) Segway_ode(t,x,params), [t_start t_end], x_init, options);

%% Save data
% filename = fullfile('saved_results','u_LMPC_dynamics_nonlinear'); 
% save(filename,'t_sim','x_sim','params');

end


%% 
function [S,M] = Compute_State_Transition_Matrices(A, B, N)
% Initialize matrices
nu = size(B,2);
nx = size(A,2);
S = zeros(nx*(N),nu*(N));
M = [];
% Build state transition matrix and linear inequality block matrices
for k=1:1:N-1
    M = [M; A^k];
    for m=0:1:k-1
        S(nx*(k-1)+1:nx*k,nu*m+1:nu*(m+1)) = A^(k-1-m)*B;
    end
end
M = [M;A^N];
for m=0:1:N-1
    S(nx*(N-1)+1:nx*N,nu*m+1:nu*(m+1)) = A^(N-1-m)*B;
end
end

function [X_goal,X_max,X_min,U_max,U_min] = ...
    Compute_Stacked_Constraints_Goals_Matrices(x_goal,x_max,x_min,u_max,u_min,N)
X_goal = repmat(x_goal,N,1);
X_max = repmat(x_max,N,1);
X_min = repmat(x_min,N,1);
U_max = repmat(u_max,N,1);
U_min = repmat(u_min,N,1);
end

function [Qstate_bar,Qctrl_bar] = Compute_Penalty_Matrices(Qstate,Qterm,Qctrl,N)
Qstate_bar = [];
Qctrl_bar = [];
for i = 1:N
    if i < N
        Qstate_bar = blkdiag(Qstate_bar,Qstate);
    end
    Qctrl_bar = blkdiag(Qctrl_bar,Qctrl);
end
Qstate_bar = blkdiag(Qstate_bar,Qterm);
end

