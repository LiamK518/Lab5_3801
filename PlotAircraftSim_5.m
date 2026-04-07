function PlotAircraftSim_5(time,aircraft_state_array,control_input_array,fig,col,label)
% Added label input for legend entries in overlayed plots

% State Variables
X = aircraft_state_array(:,1); % Inertial X Position (m)
Y = aircraft_state_array(:,2); % Inertial Y Position (m)
Z = aircraft_state_array(:,3); % Inertial Z Position (m)
psi = aircraft_state_array(:,6); % Body Yaw angle (rad)
theta = aircraft_state_array(:,5); % Body Pitch Angle (rad)
phi = aircraft_state_array(:,4); % Body Roll Angle (rad)
u_e = aircraft_state_array(:,7); % X velocity in Body Coordinates (m/s)
v_e = aircraft_state_array(:,8); % Y velocity in Body Coordinates (m/s)
w_e = aircraft_state_array(:,9); % Z velocity in Body Coordinates (m/s)
p = aircraft_state_array(:,10); % Body Roll rate (rad/s)
q = aircraft_state_array(:,11); % Body Pitch rate (rad/s)
r = aircraft_state_array(:,12); % Body Yaw rate (rad/s)

% Control Inputs
del_e = control_input_array(1,:); % Elevator deflection angle (deg)
del_a = control_input_array(2,:); % Aileron deflection angle (deg)
del_r = control_input_array(3,:); % Rudder deflection angle (deg)
del_t = control_input_array(4,:); % Thrust Coefficent 

% Make sure time is a column for plotting against n x 1 state vectors
time = time(:);

%% Figure 1: Inertial Position (X, Y, Z)
figure(fig(1));
set(gcf,'Name','Inertial Position','NumberTitle','off');

subplot(3,1,1)
plot(time,X,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('X (m)')
title('Inertial Position')
legend('show','Location','best')

subplot(3,1,2)
plot(time,Y,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('Y (m)')
legend('show','Location','best')

subplot(3,1,3)
plot(time,Z,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('Z (m)')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(1)), 'Resolution', 300);

%% Figure 2: Euler Angles (psi, theta, phi)
figure(fig(2));
set(gcf,'Name','Euler Angles','NumberTitle','off');

subplot(3,1,1)
plot(time,psi,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('\psi (rad)')
title('Euler Angles')
legend('show','Location','best')

subplot(3,1,2)
plot(time,theta,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('\theta (rad)')
legend('show','Location','best')

subplot(3,1,3)
plot(time,phi,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('\phi (rad)')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(2)), 'Resolution', 300);

%% Figure 3: Inertial velocity in body frame (u_e, v_e, w_e)
figure(fig(3));
set(gcf,'Name','Body-frame Velocities','NumberTitle','off');

subplot(3,1,1)
plot(time,u_e,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('u (m/s)')
title('Inertial Velocity in Body Frame')
legend('show','Location','best')

subplot(3,1,2)
plot(time,v_e,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('v (m/s)')
legend('show','Location','best')

subplot(3,1,3)
plot(time,w_e,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('w (m/s)')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(3)), 'Resolution', 300);

%% Figure 4: Angular rates (p, q, r)
figure(fig(4));
set(gcf,'Name','Angular Rates','NumberTitle','off');

subplot(3,1,1)
plot(time,p,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('p (rad/s)')
title('Angular Velocity')
legend('show','Location','best')

subplot(3,1,2)
plot(time,q,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('q (rad/s)')
legend('show','Location','best')

subplot(3,1,3)
plot(time,r,col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('r (rad/s)')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(4)), 'Resolution', 300);

%% Figure 5: Control inputs (Z_c, L_c, M_c, N_c)
figure(fig(5));
set(gcf,'Name','Control Inputs','NumberTitle','off');

subplot(4,1,1)
plot(time,del_e(:),col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('\delta_e (degrees)')
title('Control Inputs')
legend('show','Location','best')

subplot(4,1,2)
plot(time,del_a(:),col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('\delta_a (degrees)')
legend('show','Location','best')

subplot(4,1,3)
plot(time,del_r(:),col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('\delta_r (degrees)')
legend('show','Location','best')

subplot(4,1,4)
plot(time,del_t(:),col{2},'LineWidth',1.2,'DisplayName',label); hold on; grid on
ylabel('delta_t ')
xlabel('Time (s)')
legend('show','Location','best')

exportgraphics(gcf, sprintf('fig%d.png', fig(5)), 'Resolution', 300);

%% Figure 6: 3D trajectory (with positive height upward)
figure(fig(6));
set(gcf,'Name','3D Trajectory','NumberTitle','off');

plot3(X,Y,-Z,col{4},'LineWidth',1.2,'DisplayName',label); hold on; grid on

% Mark start and end without putting them in the legend
plot3(X(1),Y(1),-Z(1),'o','Color',col{3},'MarkerSize',8,'LineWidth',2, ...
    'HandleVisibility','off');
plot3(X(end),Y(end),-Z(end),'x','Color',col{1},'MarkerSize',8,'LineWidth',2, ...
    'HandleVisibility','off');

xlabel('X (m)')
ylabel('Y (m)')
zlabel('Height (m)')
title('3D Aircraft Path (positive up)')
legend('show','Location','best')
view(3)
axis equal
% Enforce minimum span of 20 on each axis
min_span = 20;
xl = xlim; yl = ylim; zl = zlim;
if diff(xl) < min_span
    mid = mean(xl);
    xlim([mid - min_span/2, mid + min_span/2]);
end
if diff(yl) < min_span
    mid = mean(yl);
    ylim([mid - min_span/2, mid + min_span/2]);
end
if diff(zl) < min_span
    mid = mean(zl);
    zlim([mid - min_span/2, mid + min_span/2]);
end
exportgraphics(gcf, sprintf('fig%d.png', fig(6)), 'Resolution', 300);

end