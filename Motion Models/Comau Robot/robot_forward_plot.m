%% Preamble
clc;
close all;

%% Plot 3D (Home Position)

% Joint angles
Q = [0; 0; -pi/2];

% Plotting setup
h = figure('Name','ComauRobot');
xlabel('x-axis [m]')
ylabel('y-axis [m]')
zlabel('z-axis [m]')
xlim([-1, 4])
ylim([-1, 4])
zlim([-1, 4])        

hold on;
grid on;
view(-20,20);

% Plot
plot_robotpose(Q, eye(4))
hold off;

% Save to PDF
h.PaperPositionMode = 'auto';         
fig_pos = h.PaperPosition;
h.PaperSize = [fig_pos(3) fig_pos(4)];

fig1_filename = 'robot_3d_home';
print(h,fig1_filename,'-dpdf')

%% Plot 3D (Offset)

% Joint angles
Q = [-15; -20; -55]*pi/180;

% Plotting setup
h1 = figure('Name','ComauRobot');
xlabel('x-axis [m]')
ylabel('y-axis [m]')
zlabel('z-axis [m]')
xlim([-1, 4])
ylim([-1, 4])
zlim([-1, 4])        

hold on;
grid on;
view(-20,20);

% Plot
plot_robotpose(Q, eye(4))
hold off;

% Save to PDF
h1.PaperPositionMode = 'auto';         
fig_pos = h1.PaperPosition;
h1.PaperSize = [fig_pos(3) fig_pos(4)];

fig2_filename = 'robot_3d_offset';
print(h1,fig2_filename,'-dpdf')

%% Forward Kinematics Plot

%% Joint angles
% Plotting setup
fig1 = figure('Name','ComauRobot Joint Angles');
plot(tout,q)
lx = legend('$q_1$', '$q_2$', '$q_3$');
set(lx, 'Interpreter', 'latex');
xlabel('Time [s]')
ylabel('Joint Angle [deg]')

% Save to PDF
fig1.PaperPositionMode = 'auto';         
fig_pos = fig1.PaperPosition;
fig1.PaperSize = [fig_pos(3) fig_pos(4)];

fig1_filename = 'robot_forward_joint';
print(fig1,fig1_filename,'-dpdf')

%% Tool point Position

% Plotting setup
fig2 = figure('Name','ComauRobot Tool-Point Position');
Ptx = Pt(1,:)';
Pty = Pt(2,:)';
Ptz = Pt(3,:)';

Pt_new = [Ptx,Pty,Ptz];

plot(tout,Pt_new)
title('Tool-Point Position')
lx = legend('$P_{t,x}$', '$P_{t,y}$', '$P_{t,z}$');
set(lx, 'Interpreter', 'latex');
xlabel('Time [s]')
ylabel('Tool-Point Position [m]')

% Save to PDF
fig2.PaperPositionMode = 'auto';         
fig_pos = fig2.PaperPosition;
fig2.PaperSize = [fig_pos(3) fig_pos(4)];

fig2_filename = 'robot_forward_tp';
print(fig2,fig2_filename,'-dpdf')

%% Velocity

% Plotting setup
fig3 = figure('Name','ComauRobot Tool-Point Velocity');

Ptx_t = Pt_t(1,:)';
Pty_t = Pt_t(2,:)';
Ptz_t = Pt_t(3,:)';

Pt_t_new = [Ptx_t,Pty_t,Ptz_t];

plot(tout,Pt_t_new)
title('Tool-Point Velocity')
lx = legend('$\dot{P}_{t,x}$', '$\dot{P}_{t,y}$', '$\dot{P}_{t,z}$');
set(lx, 'Interpreter', 'latex');
xlabel('Time [s]')
ylabel('Tool-Point Velocity [m/s]')

% Save to PDF
fig3.PaperPositionMode = 'auto';         
fig_pos = fig3.PaperPosition;
fig3.PaperSize = [fig_pos(3) fig_pos(4)];

fig3_filename = 'robot_forward_tp_t';
print(fig3,fig3_filename,'-dpdf')

%% Acceleration

% Plotting setup
fig4 = figure('Name','ComauRobot Tool-Point Acceleration');

Ptx_tt = Pt_tt(1,:)';
Pty_tt = Pt_tt(2,:)';
Ptz_tt = Pt_tt(3,:)';

Pt_tt_new = [Ptx_tt,Pty_tt,Ptz_tt];
Pt_tt_new(2,:) = zeros(1,3); % Removing jerk
plot(tout,Pt_tt_new)
title('Tool-Point Acceleration')
lx = legend('$\ddot{P}_{t,x}$', '$\ddot{P}_{t,y}$', '$\ddot{P}_{t,z}$');
set(lx, 'Interpreter', 'latex');
xlabel('Time [s]')
ylabel('Tool-Point Acceleration [m/s^2]')

% Save to PDF
fig4.PaperPositionMode = 'auto';         
fig_pos = fig4.PaperPosition;
fig4.PaperSize = [fig_pos(3) fig_pos(4)];

fig4_filename = 'robot_forward_tp_tt';
print(fig4,fig4_filename,'-dpdf')
