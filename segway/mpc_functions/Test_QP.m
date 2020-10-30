% Test QP Code vs. quadprog

%% Example 1
clear; clc;
H = 2*[3,1;1,1];
q = [1, 6]';
A = [-2, -3;
     -1, 0;
     0, -1];
b = [-4;0;0];
[x_quadprog,~,~,~,lam_quadprog] = quadprog(H,q,A,b)
[x_myQP,lam_myQP] = Solve_QP_DualProjectedGradient(H,q(:),A,b,[],[],[],[])

%% Example 2
clear;
H = [1,-1,1
    -1,2,-2
    1,-2,4];
f = [2;-3;1];
lb = zeros(3,1);
ub = ones(size(lb));
Aeq = ones(1,3);
beq = 1/2;
Aineq = [];
Bineq = [];
[x,~,~,~,xlam] = quadprog(H,f,[],[],Aeq,beq,lb,ub)
[x_myQP,lam_myQP] = Solve_QP_DualProjectedGradient(H,f,Aineq,Bineq,Aeq,beq,lb,ub)
