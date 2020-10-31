% Problem 3
clear;clc;
close all;

%% Choose problem
prob3 = 'e';
switch prob3
    case 'a'
        num_recomputes = 1;
        incline = false;
        slack_vars = load('slack_var_a.mat','slack_var_guess');
    case 'b'
        num_recomputes = 1;
        incline = true;
        slack_vars = load('slack_var_b.mat','slack_var_guess');
    case 'c'
        num_recomputes = 2;
        incline = true;
        slack_vars = load('slack_var_c.mat','slack_var_guess');
    case 'd'
        num_recomputes = 10;
        incline = true;
        slack_vars = load('slack_var_d.mat','slack_var_guess');
        
    case 'e'
        num_recomputes = 300;
        incline = true;
        slack_vars = load('slack_var_e.mat','slack_var_guess');
        
end

%% System parameters
dt = 0.01;
N = max(300/num_recomputes,10);
A = [1, dt; 0 1];
B = [0; dt];
dim_x = size(A,1);
dim_u = size(B,2);

% Initial Conditions
t_init = 0;
x_init = [0; 0];

% State/Control Constraints
x_goal = [5; 0];
x_max = [inf; 6];
x_min = [-inf; -6];
u_max = 10;
u_min = -10;

% State/Control Penalty Matrices
Qstate = [100, 0; 0, 1];
Qctrl = 0.001;

% road incline
if incline
    g = 9.81;
    alpha = deg2rad(0.1);
    disturbance = -[0; g*sin(alpha)];
else
    disturbance = [0; 0];
end

%% Goal, Min, Max, Block Matrices
[QP_params.X_goal, QP_params.X_max, QP_params.X_min, QP_params.U_max, QP_params.U_min] = ...
    Compute_Stacked_Constraints_Goals_Matrices(x_goal,x_max,x_min,u_max,u_min,N);

% State Transition Matrix
[QP_params.S,QP_params.M] = Compute_State_Transition_Matrices(A,B,N);

% Penalty matrices
QP_params.x_init = x_init;
QP_params.dim_u = dim_u;
QP_params.N = N;
[QP_params.Qstate_all, QP_params.Qctrl_all] = Compute_Penalty_Matrices(Qstate,Qctrl,N);

% QP matrices
[H,q,Ain,bin] = Compute_QP_Matrices(QP_params);


%% Solve QP and Simulate twice during trajectory
t_traj = t_init;
x_traj = x_init;
u_traj = [];
tspan = 3;

for j = 1:num_recomputes
    x_init = x_traj(:,end);
    t_init = t_traj(:,end);
    QP_params.x_init = x_init;
    [H,q,Ain,bin] = Compute_QP_Matrices(QP_params);
    
    % Calculate input
    %     [u_traj_qp,~,~,~,slack_var_guess(j)] = quadprog(2*H,q,Ain,bin,[],[],[],[]);
    slack_var_guess = slack_vars.slack_var_guess(j);
    [u_traj_j] = Solve_QP_DualProjectedGradient(2*H,q,Ain,bin,[],[],[],[],4000,1e-6,slack_var_guess.ineqlin);
    u_traj_j = u_traj_j';
    
    % Simulate discrete dynamics
    num_iter = max((tspan/num_recomputes)/dt,2);
    t_traj_j = t_traj(end);
    x_traj_j = x_traj(:,end);
    for i = 1:num_iter-1
        t_traj_j(:,i+1) = t_traj_j(:,i) + dt;
        x_traj_j(:,i+1) = A*x_traj_j(:,i) + B*u_traj_j(:,i) + disturbance;
    end
    
    % Store trajectories
    t_traj = [t_traj, t_traj_j(2:end)];
    x_traj = [x_traj, x_traj_j(:,2:end)];
    u_traj = [u_traj, u_traj_j(1:num_iter-1)];
end

% save lambda
% name = "slack_var_"+prob3;
% save(name,'slack_var_guess');

%% Plot trajectories
Plot_Traj(t_traj,x_traj,u_traj,prob3);

%% Animation
% Animation(x_traj,x_goal,tspan,dt);

%% State Transition Matrices
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

function [X_goal,X_max,X_min,U_max,U_min] = ...
    Compute_Stacked_Constraints_Goals_Matrices(x_goal,x_max,x_min,u_max,u_min,N)
X_goal = repmat(x_goal,N,1);
X_max = repmat(x_max,N,1);
X_min = repmat(x_min,N,1);
U_max = repmat(u_max,N,1);
U_min = repmat(u_min,N,1);
end

function [Qstate_bar,Qctrl_bar] = Compute_Penalty_Matrices(Qstate,Qctrl,N)
Qstate_bar = [];
Qctrl_bar = [];
for i = 1:N
    Qstate_bar = blkdiag(Qstate_bar,Qstate);
    Qctrl_bar = blkdiag(Qctrl_bar,Qctrl);
end
end

function [H,q,Ain,bin] = Compute_QP_Matrices(params)
% Extract params
N = params.N;
X_goal = params.X_goal;
X_max = params.X_max;
X_min = params.X_min;
U_max = params.U_max;
U_min = params.U_min;
S = params.S;
M = params.M;
Qstate_all = params.Qstate_all;
Qctrl_all = params.Qctrl_all;
x_init = params.x_init;
dim_u = params.dim_u;

% compute QP matrices
H = S'*Qstate_all*S + Qctrl_all;
q = (2*(M*x_init - X_goal)'*Qstate_all*S)';
Ain = [S;
    -S;
    eye(N*dim_u);
    -eye(N*dim_u)];
bin = [X_max - M*x_init;
    -X_min + M*x_init;
    U_max;
    -U_min];
end

% Plot Trajectories
function [] = Plot_Traj(t_traj,x_traj,u_traj,prob3)
wdline = 5;
sz = 20;
ft = 30;
figure;
subplot(131);
scatter(t_traj,x_traj(1,:),sz);
hold on; yline(5,'g','LineWidth',wdline);
switch prob3
    case 'a'
    case 'b'
    case 'c'
        hold on; xline(1.5,'k','LineWidth',wdline/2);
    case 'd'
        for k = 1:9
            hold on; xline(k*0.3,':k','LineWidth',wdline/5);
        end
end
xlabel('t: time [sec]');
ylabel('p: car position [m]');
legend('Position Trajectory',...
       'Position Goal');
set(gca,'FontSize',ft);

subplot(132);
scatter(t_traj,x_traj(2,:),sz);
hold on; yline(6,'--r','LineWidth',wdline);
hold on; yline(0,'g','LineWidth',wdline);
switch prob3
    case 'a'
    case 'b'
    case 'c'
        hold on; xline(1.5,'k','LineWidth',wdline/2);
    case 'd'
        for k = 1:9
            hold on; xline(k*0.3,':k','LineWidth',wdline/5);
        end
end
xlabel('t: time [sec]');
ylabel('v: car velocity [m/s]');
legend('Velocity Trajectory',...
       'Max Velocity Bound',...
       'Velocity Goal');
set(gca,'FontSize',ft);

subplot(133);
hold on; yline(10,'--r','LineWidth',2);
hold on; yline(-10,'--r','LineWidth',2);
hold on; scatter(t_traj(1:end-1),u_traj,sz);
switch prob3
    case 'a'
    case 'b'
    case 'c'
        hold on; xline(1.5,'k','LineWidth',wdline/2);
    case 'd'
        for k = 1:9
            hold on; xline(k*0.3,':k','LineWidth',wdline/5);
        end
    case 'e'
        yline(0,'k','Linewidth',1);
end
xlabel('t: time [sec]');
ylabel('u: car acceleration [m/s^2]');
legend('Max Acceleration Bound',...
       'Min Acceleration Bound',...
       'Control Input (Acceleration) Trajectory');
set(gca,'FontSize',ft);
end

function [] = Animation(x_traj,x_goal,tspan,dt)
figure
com = x_traj(1,1);
car_width = 10;
car_height = 0.3;
car_color = [1 203/255 5/255];
wheel_rad = 5;
wheel_height = 0.4;
wheel_color = [0 39/255 76/255];
wheel_rear = rectangle('Position',[com-0.6*car_width, 0, wheel_rad, wheel_height/2],...
    'Curvature',[1 1],...
    'FaceColor',wheel_color);
wheel_front = rectangle('Position',[com+0.1*car_width, 0, wheel_rad, wheel_height/2],...
    'Curvature',[1 1],...
    'FaceColor',wheel_color);
car = rectangle('Position', [com-car_width/2, car_height/2, car_width, car_height],...
    'Curvature',[0.5 0.5],...
    'FaceColor',car_color);
hold on;
com_plot = scatter(com,car_height,100,wheel_color,'filled');
goal_plot = scatter(x_goal(1),car_height,100,'g','filled');
axis([-10 30 0 1]);
for j = 1: tspan/dt
    com = x_traj(1,j);
    set(wheel_rear,'Position',[com-0.6*car_width, 0, wheel_rad, wheel_height/2]);
    set(wheel_front,'Position',[com+0.1*car_width, 0, wheel_rad, wheel_height/2]);
    set(car,'Position', [com-0.5*car_width, car_height/2, car_width, car_height]);
    set(com_plot,'XData',com);
    drawnow
    pause(0.001);
end
end







