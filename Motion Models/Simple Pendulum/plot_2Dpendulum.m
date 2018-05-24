close all;
clc;

fig1 = figure('Name','StewartPlatform Animation');
xlabel('x-axis [m]')
ylabel('z-axis [m]')
xlim([-3, 3])
        ylim([-3, 1])
        hold on;
        grid on;
% Tool Point
Pt = [0;
	  0];
Pp = [0.5176,-1.932];
      
plot_line(Pt, Pp);
plot(Pp(1), Pp(2), 'ro', 'LineWidth', 2);

% Save to PDF
fig1.PaperPositionMode = 'auto';         
fig_pos = fig1.PaperPosition;
fig1.PaperSize = [fig_pos(3) fig_pos(4)];

fig1_filename = 'pendulum_robot_3d_home';
print(fig1,fig1_filename,'-dpdf')