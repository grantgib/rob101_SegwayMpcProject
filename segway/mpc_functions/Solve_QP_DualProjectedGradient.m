function [U, lambda] = Solve_QP_DualProjectedGradient(H,q,G,Wtilde,lam0,maxIter)
%% Solve_QP_DualProjectedGradient - Implements type of QP solver
%   Dual projected gradient algorithm for solving QP problem
%   Original problem:
%       Min 1/2*U'*H*U + q'*U 
%           s.t. A*U <= b
%
%   Dual function problem
%       Min 1/2*lam'*H_dual*lam + q_dual*lam 
%           s.t. lam >= 0
    if nargin < 6
        maxIter = 30;
    end
    H_dual = (G/H)*G';              % dual quadratic cost matrix
    q_dual = (G/H)*q + Wtilde;      % dual linear cost vector
    H_dual_norm = norm(H_dual);     % upper bound on eigenvalues of H_dual
    
    lambda = lam0;                             % initialize dual variable guess
    dual_gradient = H_dual*lambda + q_dual;    % gradient of dual cost function
    for k = 1:maxIter
        % iterate with gradient descent method. max is used to project the
        % dual variables back into the acceptable set (nonnegative orthant)
        lambda = max(0,lambda-(1/H_dual_norm)*dual_gradient); 
        dual_gradient = H_dual*lambda + q_dual;
    end
    
    U = -H\(G'*lambda + q);    % primal variable solution
    return;
end

