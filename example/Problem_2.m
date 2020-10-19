% Problem 2
clear;
clc;

dt = 0.1;
N = 3;
A = [1, dt; 0 1];
B = [0; dt];


[S,M] = Compute_State_Transition_Matrices(A,B,N)

% Check matrices with state trajectory from Example 1
x0 = [0;2];
U_traj = 3*ones(N,1);
X_state_transition = S * U_traj + M*x0

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







