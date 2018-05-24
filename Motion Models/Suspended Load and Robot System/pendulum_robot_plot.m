%% Preamble
clc;
close all;

%% Plot 3D (Home Position)

% Joint angles
q = [0; 0; -pi/2];

% Tool Point
pt = [2.6192;
	  0;
      2.2400];
  
% Pendulum  
pp = [2.6192;
	  0;
      0.2400];

% Plotting setup
fig1 = figure('Name','Comau Robot and Pendulum');  
xlabel('x-axis [m]')
ylabel('y-axis [m]')
zlabel('z-axis [m]')
xlim([-1, 4])
ylim([-1, 4])
zlim([-1, 4])
hold on;
grid on;
view(-30,20);

% Plot
plot_robotpose(q, eye(4))
plot_pendulum(pt, pp, eye(4));
hold off;

% Save to PDF
fig1.PaperPositionMode = 'auto';         
fig_pos = fig1.PaperPosition;
fig1.PaperSize = [fig_pos(3) fig_pos(4)];

fig1_filename = 'pendulum_robot_3d_home';
print(fig1,fig1_filename,'-dpdf')

%% Plot 3D (Home Position)

% Joint angles
q = Q(26500,:,:);

% Tool Point
pt = Pt(:,:,26500);
pt_tt = Pt_tt(:,:,26500);

% Pendulum
pp = Pp(26500,:)';

% Plotting setup
fig2 = figure('Name','Comau Robot and Pendulum');  
xlabel('x-axis [m]')
ylabel('y-axis [m]')
zlabel('z-axis [m]')
xlim([-1, 4])
ylim([-1, 4])
zlim([-1, 4])
hold on;
grid on;
view(-30,20);

% Plot
plot_robotpose(q, eye(4))
plot_pendulum(pt, pp, eye(4));
hold off;

% Save to PDF
fig2.PaperPositionMode = 'auto';         
fig_pos = fig2.PaperPosition;
fig2.PaperSize = [fig_pos(3) fig_pos(4)];

fig2_filename = 'pendulum_robot_3d_offset';
print(fig2,fig2_filename,'-dpdf')