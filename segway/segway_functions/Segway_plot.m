function [] = Segway_plot(t_sim,x_sim,params)
%% Extract control inputs and compute additional variables
u_sim = Extract_Torques(t_sim,x_sim,params);
absolute_segway_angle = [1 1 0 0]*x_sim';
absolute_segway_angular_velocity = [0 0 1 1]*x_sim';

%% Plot settings
font_title = 18;
font_label = 16;
line_width = 2.5;

%% Plot state trajectories
figure
state_header = ["$\phi$","$\theta$","$\dot{\phi}$","$\dot{\theta}$"];
label_header = ["Angle [rad]","Ang. Velocity [rad/s]","Angle [rad]","Ang. Velocity [rad/s]"];
for i = 1:4
    subplot(2,2,i)
    plot(t_sim,x_sim(:,i),'linewidth',line_width);
    title(state_header(i),'interpreter','latex','fontsize',font_label);
    xlabel('Time [s]','interpreter','latex','fontsize',font_label);
    ylabel(label_header(i),'interpreter','latex','fontsize',font_label);
    grid on;
end
sgtitle('State Trajectories','interpreter','latex','fontsize',font_title);

%% Plot input trajectories
figure
plot(t_sim,u_sim,'linewidth',line_width);
xlabel('Time [s]','interpreter','latex','fontsize',font_label);
ylabel('Torque [N-m]','interpreter','latex','fontsize',font_label);
title('$U$ (Control Input) Trajectories','interpreter','latex','fontsize',font_title);
grid on;

%% Plot Absolute Angle/Angular Velocity Trajectories
figure
subplot(1,2,1);
plot(t_sim,absolute_segway_angle,'linewidth',line_width);
title('Absolute Angle','interpreter','latex','fontsize',font_label);
xlabel('Time [s]','interpreter','latex','fontsize',font_label);
ylabel("Angle [rad]",'interpreter','latex','fontsize',font_label);
grid on;
subplot(1,2,2);
plot(t_sim,absolute_segway_angular_velocity,'linewidth',line_width);
title('Absolute Angular Velocity','interpreter','latex','fontsize',font_label);
xlabel('Time [s]','interpreter','latex','fontsize',font_label);
ylabel('Angular Velocity [rad/s]','interpreter','latex','fontsize',font_label);
sgtitle('Absolute Angle & Angular Velocity Trajectories','fontsize',font_title);
grid on;
end