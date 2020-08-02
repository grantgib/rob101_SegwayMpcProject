function dxdt = Segway_ode(t,x,params)
%% SEGWAY_ODE - Compute ODE for segway model
%   This function takes in the current time, state, and params in order to
%   calculate the differential equation for the segway. 

%% Extract params data
K = params.K;
ctrl_type = params.ctrl_type;
sim_type = params.sim_type;

%% Simulate using linear/nonlinear model and linear/nonlinear controller
if sim_type == "LINEAR"
    A = [0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
    B = [0;0;-0.7172;1.6744];
    
    if ctrl_type == "LINEAR"
        u = K*x;
        dxdt = A*x + B*u;
    elseif ctrl_type == "LMPC"
        u = Compute_MPC_Control(A,B);
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
        u = Compute_MPC_Control_MPC(A,B);
    end
    dxdt(1:2) = x(3:4);
    dxdt(3:4) = D\(-C*dq - G + B*u);
end

end

