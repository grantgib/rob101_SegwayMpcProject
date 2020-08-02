function [tout, xout] = Segway_simulate(t_end,x0)
%% Segway_simulate - Generate state trajectory for segway given initial condition
%   Detailed description if necessary

%% Linear Plant Model
A = [0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
B = [0;0;-0.7172;1.6744];

%% Control type
ctrl_type = "LINEAR"; % LINEAR, NONLINEAR, LMPC
sim_type = "LINEAR"; % LINEAR, NONLINEAR"

%% Pole placement and LQR for Linear State Feedback Control (I think we can get rid of this eventually)
fdbk_type = "lqr";
if fdbk_type == "pp" % Pole placement method to be covered in EECS 560
    P = [-2.5849, -1.2387 , -0.4108 + 0.2230i,  -0.4108 - 0.2230i]; % desired e-values
    K = -place(A,B,P);
else % A more powerful method from EECS 565
    K = -lqr(A,B,diag([1 1 100 1]),1);
%     disp(eig(A+B*K));
end

%% Forward Integration Settings
refine=4;
RelTol = 10^-7;
AbsTol = 10^-8;
options = odeset('Refine',refine, 'RelTol',RelTol,'AbsTol',AbsTol);
t_start=0;
params = struct('K',            K,...
                'sim_type',     sim_type,...
                'ctrl_type',    ctrl_type);
            
%% Simulate with Forward Integration (Runge Kutta Fehlberg 45)
[tout, xout] = ode45(@(t,x) Segway_ode(t,x,params), [t_start t_end], x0, options);

end

