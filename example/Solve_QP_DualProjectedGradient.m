function [U, slack_var] = Solve_QP_DualProjectedGradient(H,q,Aineq,Bineq,Aeq,Beq,lb,ub,maxIter,tol,slack_var_guess)
%% Solve_QP_DualProjectedGradient - Implements type of QP solver
%   Dual projected gradient algorithm for solving QP problem
%   Original problem:
%       Min 1/2*U'*H*U + q'*U
%           s.t.
%                -I*U <= -lb
%                 I*U <= ub
%             Aineq*U <= Bineq
%                Aeq*U = Beq
%
%   Rearranged problem:
%       Min 1/2*U'*H*U + q'*U
%           s.t. G*U <= W
%           where G = [-I; I; Aineq; Aeq]
%           where W = [-lb; ub; Bineq; Beq]
%
%   Dual function problem
%       Min 1/2*slack_var'*H_dual*slack_var + q_dual*slack_var
%           s.t. slack_var_ineq >= 0
%
%       Note: slack_var_ineq =
%           slack_var(1:length(lb)+length(ub)+length(Aineq))
%
if nargin < 9
    maxIter = 4000;
    tol = 1e-6;
end
dim_U = size(H,2);
% Concatenate constraints
G = [];
W = [];
if ~isempty(lb)
    G = -eye(dim_U);
    W = -lb;
end
if ~isempty(ub)
    G = [G; eye(dim_U)];
    W = [W; ub];
end
G = [G; Aineq; Aeq];
W = [W; Bineq; Beq];
dim_lmd = length(W)-length(Beq);
% Minimize the negative of the dual function
H_dual = (G/H)*G';                              % dual quadratic cost matrix
q_dual = (G/H)*q + W;                           % dual linear cost vector
H_dual_norm = norm(H_dual);                     % upper bound on eigenvalues of H_dual
if nargin < 11
    slack_var = ones(length(W),1);                  % initialize dual variable guess (greater than zero for all entries); slack_var=[lambda;p]; lambda-inequality slack var; p-equality slack var
else
    slack_var = slack_var_guess;
end
    
dual_gradient = H_dual*slack_var + q_dual;      % gradient of dual cost function
for k = 1:maxIter
    % iterate with gradient descent method. max is used to project the
    % dual variables back into the acceptable set (nonnegative orthant)
    slack_var_past = slack_var;
    slack_var_temp = slack_var-(1/H_dual_norm)*dual_gradient;
    slack_var = [max(0,slack_var_temp(1:dim_lmd)); slack_var_temp(dim_lmd+1:end)] ;     %Per KKT conditions: only enforce dual variable inequality constraint(lambda >=0). p=0 can be any value for optimization
    dual_gradient = H_dual*slack_var + q_dual;
    if norm(slack_var - slack_var_past) < tol
        break
    end
end

U = -H\(G'*slack_var + q);    % primal variable solution
return;
end
%