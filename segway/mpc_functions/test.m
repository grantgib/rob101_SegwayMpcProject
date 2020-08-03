H = 2*[3,1;1,1];
q = [1, 6]';
A = [-2, -3;
     -1, 0;
     0, -1];
b = [-4;0;0];

[x_quadprog,~,~,~,lam_quadprog] = quadprog(H,q,A,b)

[x_myQP,lam_myQP] = Solve_QP(H,q(:),A,b,[1;1;1])