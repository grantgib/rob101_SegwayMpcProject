% Top level script for running the Segway model
% 
% In the file Segway_ODE45, you can select to simulate a 
% nonlinear model or a linear model
%

% Set Initial Conditon
x0=[0;10;0;0] ; %[phi, theta, dphi/dt, dtheta/dt]
t_end=25;
[tout, xout]=Segway_ODE45(t_end,x0);

% Plot various variables and then run an animation

cart_position = [1 1 0 0]*xout';
cart_velocity = [0 0 1 1]*xout';
phi = [1 0 0 0]*xout';
theta = [0 1 0 0]*xout';

figure(1);
plot(tout,cart_position,'linewidth',2.5);
xlabel('Time (Sec)','fontsize',16);
ylabel('p (m)','fontsize',16);
title('Segway Cart Position','fontsize',18);
grid on
figure(2);
plot(tout,cart_velocity,'linewidth',2.5);
xlabel('Time (Sec)','fontsize',16);
ylabel('dp/dt (m/s)','fontsize',16);
title('Segway Cart Velocity','fontsize',18);
grid on

figure(3);
plot(tout,phi,'linewidth',2.5);
xlabel('Time (Sec)','fontsize',16);
ylabel('\phi (rad)','fontsize',16);
title('Segway Pendulum Angle','fontsize',18);
grid on
figure(4);
plot(tout,theta,'linewidth',2.5);
xlabel('Time (Sec)','fontsize',16);
ylabel('\theta (rad)','fontsize',16);
title('Segway Wheel Angle','fontsize',18);
grid on


% Animation to see what is happening
pause(1)

Segway_anim(tout,xout(:,1),xout(:,2));

% Animate again, this time in slow motion

pause(1)
Segway_anim(tout,xout(:,1),xout(:,2),.01);



