function [H, L, G, W, T, IMPC] = Form_QP_Matrices(A, B, Q, R, P, xlim, ulim, N)
% Form matrices needed for QP in LQ-MPC
% Minimize 1/2*U^T H U + q^{\rm T} U subject to G U <= W +T*x_0
% where q = L*x_0
% The matrix IMPC extracts the first control move, u_MPC=IMPC*U
% N is the horizon,xlim, ulim are limits on x and u910nx = size(A,1);
nu = size(B,2);
Qbar = [];
Rbar = [];
S = zeros(nx*(N),nu*(N));
M = [];
Xmax = [ ];
Xmin = [ ];
Umax = [ ];
Umin = [ ];
for k=1:1:N-1
Qbar = blkdiag(Qbar, Q); % blkdiag - useful command
Rbar = blkdiag(Rbar, R);
M = [M; A^k];
for m=0:1:k-1
S(nx*(k-1)+1:nx*k,nu*m+1:nu*(m+1)) = A^(k-1-m)*B;
end
Xmax = [Xmax; xlim.max'];
Xmin = [Xmin; xlim.min'];
Umax = [Umax; ulim.max'];
Umin = [Umin; ulim.min'];
end
Qbar = blkdiag(Qbar,P);
Rbar = blkdiag(Rbar,R);
M = [M;A^N];
for m=0:1:N-1
S(nx*(N-1)+1:nx*N,nu*m+1:nu*(m+1)) = A^(N-1-m)*B;
end
Xmax = [Xmax; xlim.max'];
Xmin = [Xmin; xlim.min'];
Umax = [Umax; ulim.max'];
Umin = [Umin; ulim.min'];
Htemp = S'*Qbar*S + Rbar;
Id = [eye(nu*(N),nu*(N))];
G = [S;-S;Id;-Id];
W = [Xmax;-Xmin;Umax;-Umin];
T = [-M;M;zeros(nu*N,nx);zeros(nu*N,nx)];
Ltemp = S'*Qbar*M;
IMPC = zeros(nu, nu*N);
IMPC(1:nu,1:nu) = eye(nu,nu);

H = 2*Htemp;    % Because of original cost function derivation
L = 2*Ltemp;
end

