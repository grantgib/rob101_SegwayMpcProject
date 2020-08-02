function [A_d,B_d] = Discretize_Dynamics(dyn_struct, Ts)
%DISCRETIZE_DYNAMICS - Discretize linear plant dynamics using zero order
%hold method. The differential equation and corresponding difference 
%equation are shown below.

% Differential Equation:    dx/dt = A_c*x + B_c*u
% Difference Equation:      x_{k+1} = A_d*x_{k} + B_d*u_{k}

%% Extract Input Data
A_c = dyn_struct.A_c;   % Continuous-time state matrix
B_c = dyn_struct.B_c;   % Continuous-time input matrix

dim_state = size(dyn_struct.A_c,1); % dimension of state variables

% Output matrix
if ~isfield(dyn_struct,'C_c')
    C_c = eye(dim_state);
else
    C_c = dyn_struct.C_c; 
end

% Direct-transition (or feedthrough) matrix
if ~isfield(dyn_struct,'C_c')
    D_c = 0;
else
    D_c = dyn_struct.D_c;
end

%% Discretize system with Zero Order Hold method
sysc = ss(A_c,B_c,C_c,D_c);   % create continuous-time state space model
sysd = c2d(sysc,Ts,'zoh');    % create discrete-time state space model

A_d = sysd.A;                 % Discrete-time state matrix
B_d = sysd.B;                 % Discrete-time input matrix

end


