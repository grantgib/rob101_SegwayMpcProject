function [U, lam] = Solve_QP(H,q,G,Wtilde,lam0,maxIter)
% Implements dual projected gradient algorithm for solving QP problem
%   Min 1/2*U'*H*U + q'*U s.t. A*U <= b
    if nargin < 6
        maxIter = 30;
    end
    G_invH = G/H;
    Hd = G*inv(H)*G';
    qd =G*inv(H)*q + Wtilde;
    lam = lam0;
    Hd_norm = norm(Hd);
    k = 1;
    df = Hd*lam + qd;
    while k <= maxIter
        lam = max(0,lam-(1/Hd_norm)*df);
        df = Hd*lam + qd;
        k = k + 1;
    end
    U = -inv(H)*(G'*lam + q);
    return;
end

