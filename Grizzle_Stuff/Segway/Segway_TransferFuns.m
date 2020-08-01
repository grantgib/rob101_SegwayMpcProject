%
% Segway_TransferFuns.m 
%
A=[0 0 1 0; 0 0 0 1; 2.568 0 0 0; -5.020  0 0 0];
B=[0;0;-0.7172;1.6744];


if 0
    C=[1 0 0 0]
    Segway = ss(A,B,C,0)  %% D=0 as in our model, then enter a $0$ for $D$.
    [num,den]= tfdata(Segway,'v')
    [z,p,k] = zpkdata(Segway)
elseif 1
        C=[0 1 0 0]
    Segway = ss(A,B,C,0)  %% D=0 as in our model, then enter a $0$ for $D$.
    [num,den]= tfdata(Segway,'v')
    [z,p,k] = zpkdata(Segway)
end