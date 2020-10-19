% Problem 3
clear;clc;
close all;
% Given
dt = 0.1;
N = 10;
A = [1, dt; 0 1];
B = [0; dt];
nx = size(A,1);
x0 = [0; 2];
x_goal = [0; 5];
x_max = [inf; 10];
Qstate = [0, 0; 0, 100];
Qctrl = 1;

% Goal, Min, Max, Block Matrices
X_goal = repmat(x_goal,N,1);
X_max = repmat(x_max,N,1);

% State Transition Matrix
[S,M] = Compute_State_Transition_Matrices(A,B,N);

% Penalty matrices
[Qstate_all, Qctrl_all] = Compute_Penalty_Matrices(Qstate,Qctrl,N);

% QP matrices
H = S'*Qstate_all*S + Qctrl_all;
q = (2*(M*x0 - X_goal)'*Qstate_all*S)';
Ain = S;
bin = [X_max - M*x0];
[U] = Solve_QP_DualProjectedGradient(H,q,Ain,bin,[],[],[],[])

% Check state trajectory with inputs
X_traj = reshape(S*U + M*x0,nx,N);
figure;
subplot(121);
plot(X_traj(1,:)); 
subplot(122);
plot(X_traj(2,:));


%% Functions
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

function [Qstate_all,Qctrl_all] = Compute_Penalty_Matrices(Qstate,Qctrl,N)
Qstate_all = [];
Qctrl_all = [];
for i = 1:N
    Qstate_all = blkdiag(Qstate_all,Qstate);
    Qctrl_all = blkdiag(Qctrl_all,Qctrl);
end

end