function [t_sim,x_sim,params] = Segway_simulate(t_end,x0)
%% Segway_simulate - Generate state trajectory for segway given initial condition

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
Q = diag([1 1 100 1]);              % State penalty diagonal matrix
R = 1;                              % Input penalty matrix (only one input so 1x1)
[~,Q_term] = dlqr(A_d,B_d,Q,R);     % Terminal state penalty

xlim = struct('max',    [inf,inf,inf,inf],...
              'min',    -[inf,inf,inf,inf]);    % state limits
ulim = struct('max',    inf,...
              'min',    -inf);                  % input/control limits

% Minimize 1/2*U'*H*U + q'*U subject to G*U <= W + T*x_0
%   where q = L*x_0
%   IMPC extracts the first control move, u_MPC = IMPC*U
[H, L, G, W, T, IMPC] = Form_QP_Matrices(A_d, B_d, Q, R, Q_term, xlim, ulim, N);

%% Forward Integration Settings
options = odeset('Refine',4,'RelTol',1e-7,'AbsTol',1e-8);
t_start = 0;
params = struct('sim_type',     sim_type,...
                'ctrl_type',    ctrl_type,...
                'H',            H,...
                'L',            L,...
                'G',            G,...
                'W',            W,...
                'T',            T,...
                'IMPC',         IMPC,...
                'N',            N);
            
%% Simulate with desired Forward Integration Scheme(Runge Kutta Fehlberg 45)
[t_sim, x_sim] = ode45(@(t,x) Segway_ode(t,x,params), [t_start t_end], x0, options);

%% Save data
% filename = fullfile('saved_results','u_LMPC_dynamics_nonlinear'); 
% save(filename,'t_sim','x_sim','params');

end

