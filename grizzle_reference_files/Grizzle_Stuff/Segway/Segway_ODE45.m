function [tout, xout]=Segway_ODE45(t_end,x0)
%% Linear Plant Model
A = [0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
B = [0;0;-0.7172;1.6744];

%% Control type
ctrl_type = "LMPC"; % LINEAR, NONLINEAR, LMPC
sim_type = "LINEAR"; % LINEAR, NONLINEAR"

%% Pole placement and LQR for Linear State Feedback Control
fdbk_type = "pp";
if fdbk_type == "pp" % Pole placement method to be covered in EECS 560
    P = [-2.5849, -1.2387 , -0.4108 + 0.2230i,  -0.4108 - 0.2230i]; % desired e-values
    K = -place(A,B,P);
else % A more powerful method from EECS 565
    K = -lqr(A,B,diag([1 1 100 1]),1);
    disp(eig(A+B*K));
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
[tout, xout] = ode45(@(t,x) f(t,x,params), [t_start t_end], x0, options);
end

%% Compute differential equation for forward integration
function dxdt = f(t,x,params)
K = params.K;
ctrl_type = params.ctrl_type;
sim_type = params.sim_type;

% Simulate using linear model and linear controller
if sim_type == "LINEAR" 
    A = [0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
    B = [0;0;-0.7172;1.6744];
    
    if ctrl_type == "LINEAR"
        u = K*x;
        dxdt = A*x + B*u;
    elseif ctrl_type == "LMPC"
        u = Control_MPC(A,B);
        dxdt = A*x + B*u;
    end
    
% Simulate NL model
else
    dxdt = zeros(4,1);
    q = x(1:2);
    dq = x(3:4);
    [D,C,G,B]= dyn_mod_segway(q,dq);
    if ctrl_type == "LINEAR"
        u = K(1)*x(1) + K(2)*x(2) + K(3)*x(3) + K(4)*x(4);
    elseif ctrl_type == "NONLINEAR"
        u = K(1)*x(1) + K(2)*x(2)/(1+.1*abs(x(2))) + K(3)*x(3) + K(4)*x(4)/(1+.1*abs(x(4)));
    elseif ctrl_type == "LMPC"
        A = [0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
        B = [0;0;-0.7172;1.6744];
        u = Control_MPC(A,B);
    end
    dxdt(1:2) = x(3:4);
    dxdt(3:4) = D\(-C*dq - G + B*u);
end

end
