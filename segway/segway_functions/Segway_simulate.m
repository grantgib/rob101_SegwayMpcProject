function [tout, xout] = Segway_simulate(t_end,x0)
%% Segway_simulate - Generate state trajectory for segway given initial condition
%   Detailed description if necessary

%% Linear Plant Model
A = [0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
B = [0;0;-0.7172;1.6744];

%% Control type
ctrl_type = "LMPC"; % LINEAR, NONLINEAR, LMPC
sim_type = "NONLINEAR"; % LINEAR, NONLINEAR"

%% Discretize Linear Dynamics
Ts = 0.1;   % time step
dyn_struct = struct('A_c',  A,...
                    'B_c',  B);
[A_d, B_d] = Discretize_Dynamics(dyn_struct,Ts);

%% Formulate matrices for Quadratic Program Solver
Q = diag([1 1 100 1]);
R = 1;
[~,P] = dlqr(A_d,B_d,Q,R);
N = 5;
xlim.max = [inf, inf, inf, inf];
xlim.min = -xlim.max;
ulim.max = inf;
ulim.min = -ulim.max;
[H, L, G, W, T, IMPC] = Form_QP_Matrices(A_d, B_d, Q, R, P, xlim, ulim, N);

%% Pole placement and LQR for Linear State Feedback Control (I think we can get rid of this eventually)
fdbk_type = "lqr";
if fdbk_type == "lqr"
% A more powerful method from EECS 565
    K = -lqr(A,B,diag([1 1 100 1]),1);
%     disp(eig(A+B*K));
end

%% Forward Integration Settings
refine=4;
RelTol = 10^-7;
AbsTol = 10^-8;
options = odeset('Refine',refine,'RelTol',RelTol,'AbsTol',AbsTol);
t_start=0;
params = struct('K',            K,...
                'sim_type',     sim_type,...
                'ctrl_type',    ctrl_type,...
                'H',            H,...
                'L',            L,...
                'G',            G,...
                'W',            W,...
                'T',            T,...
                'IMPC',         IMPC,...
                'N',            N);
            
%% Simulate with Forward Integration (Runge Kutta Fehlberg 45)
[tout, xout] = ode45(@(t,x) Segway_ode(t,x,params), [t_start t_end], x0, options);

%% Save data
if ctrl_type == "LMPC"
    filename = fullfile('saved_results','u_LMPC_dynamics_nonlinear'); 
elseif ctrl_type == "LINEAR"
    filename = fullfile('saved_results','u_Linear_dynamics_nonlinear');
end
save(filename,'tout','xout','params');

end

