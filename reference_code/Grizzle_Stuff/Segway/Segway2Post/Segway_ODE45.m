function [tout, xout]=Segway_ODE45(t_end,x0)
%
%
A=[0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
B=[0;0;-0.7172;1.6744];
if 1 % Pole placement method to be covered in EECS 560
    P=[-2.5849, -1.2387 , -0.4108 + 0.2230i,  -0.4108 - 0.2230i]; % desired e-values
    K=-place(A,B,P);
else % A more powerful method from EECS 565
    K=-lqr(A,B,diag([1 1 100 1]),1), eig(A+B*K)
end
refine=4;
RelTol = 10^-7;
AbsTol = 10^-8;
options = odeset('Refine',refine, 'RelTol',RelTol,'AbsTol',AbsTol);
t_start=0;
[tout, xout] = ode45(@f, [t_start t_end], x0, options, K);
function dx=f(t,x,K)
%
if 1 % Simulate using linear model and linear controller
    A=[0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
    B=[0;0;-0.7172;1.6744];
    dx = (A+B*K)*x;
    %
else  % Simulate NL model
    dx=zeros(4,1);
    q=x(1:2);
    dq=x(3:4);
    [D,C,G,B]= dyn_mod_segway(q,dq);
    if 1 % linear controller
        u = K(1)*x(1) + K(2)*x(2) + K(3)*x(3) + K(4)*x(4);
    else
        u = K(1)*x(1) + K(2)*x(2)/(1+.1*abs(x(2))) + K(3)*x(3) + K(4)*x(4)/(1+.1*abs(x(4)));
    end
    dx(1:2)=x(3:4);
    dx(3:4)=inv(D)*(-C*dq - G + B*u);
end
return
