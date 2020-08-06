function [u_mpc] = Compute_MPC_Control(A,B,params)
%CONTROL_MPC - This function computes the Model Predictive Control input
%given a linear plant model

%% Extract Input Data

x0 = params.x0;
H = params.H;
G = params.G;
L = params.L;
W = params.W;
T = params.T;
IMPC = params.IMPC;



%% Solve QP - control input sequence solution
% % Minimize 1/2*U^T H U + q^{\rm T} U subject to G U <= W +T*x_0 = Wtilde
% % where q = L*x_0
% q = L*x0;
% Wtilde = W + T*x0;
% lam0 = ones(length(W),1);
% % [U, lam] = Solve_QP(H,q,G,Wtilde,lam0);
% 
% 
% %% Apply first control input in the solution sequence
% % u_mpc = IMPC*U;


%% QUADPROG Solution
% % Minimize 1/2*U^T H U + q^{\rm T} U subject to G U <= W +T*x_0 = Wtilde
% % where q = L*x_0
q = L*x0;
Wtilde = W + T*x0;
options=optimoptions(@quadprog,'Display','off');
[U,cost,EXITFLAG] = quadprog(H,q,G,Wtilde,[],[],[],[],[],options);
if EXITFLAG ~= 1
    disp('NOT WORKING');
    pause;
end
u_mpc = U(1);

end

