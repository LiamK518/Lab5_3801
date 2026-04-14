% Lab 5 Script
clear all;
clc;
addpath('/Users/parkerhimes/Documents/MATLAB/ASEN_3801/Lab_1'); % used to access stdatmo
aircraft_parameters = ttwistor();

%% Task 2
% Declaring constant inputs across simulations
wind_inertial = zeros(3,1);




% Part 1
aircraft_state_0_2_1 = zeros(12,1);
aircraft_state_0_2_1(3,1) = -1609.34; % Altitude
aircraft_state_0_2_1(7,1) = 21; %m/s (airspeed)
aircraft_surfaces_2_1 = zeros(4,1);
t_span_2_1 = [0,400];
[t_2_1,aircraft_state_2_1] = ode45(@(time,aircraft_state)AircraftEOM(time, aircraft_state, aircraft_surfaces_2_1,wind_inertial, aircraft_parameters),t_span_2_1,aircraft_state_0_2_1);

% Declaring components for plotting
fig_2_1=[211,212,213,214,215,216]; % Figure naming vector for task 1 part 2 (1_2)
col = {'r', 'b', 'g', 'k'}; % Color vector
% Control input array 
aircraft_surfaces_2_1_plot = ones(4,length(t_2_1));
aircraft_surfaces_2_1_plot = aircraft_surfaces_2_1 .* aircraft_surfaces_2_1_plot;
PlotAircraftSim_5(t_2_1,aircraft_state_2_1,aircraft_surfaces_2_1_plot,fig_2_1,col,'2.1');





%% part 2
% Declaring initial states
% Breaking up the state into the 4 state vectors
% Position
pos_2 = [ 0 0 -1800]; % meters 

% Euler Angles
euler_2 = [0, 0.0278, 0]; % All in radians already

% Body Velocity
velo_2 = [20.99, 0, 0.5837 ]; % All in m/s

% Rotation rates
rates_2 = [0.08, -0.2,0]; % In degrees / second 
rates_2 = deg2rad(rates_2); % rad/ second

aircraft_state_0_2_2 = [pos_2,euler_2,velo_2,rates_2]';

% Aircraft Surfaces
aircraft_surfaces_2_2 = [0.1079,0,0,0.3182]'; % Radians

t_span_2_2 = [0,1000];

[t_2_2,aircraft_state_2_2] = ode45(@(time,aircraft_state)AircraftEOM(time, aircraft_state, aircraft_surfaces_2_2, wind_inertial, aircraft_parameters),t_span_2_2,aircraft_state_0_2_2);

% Declaring components for plotting
fig_2_2= fig_2_1 + 10; % Figure naming vector for task 1 part 2 (1_2)
% Control input array 
aircraft_surfaces_2_2_plot = ones(4,length(t_2_2));
aircraft_surfaces_2_2_plot = aircraft_surfaces_2_2_plot .* aircraft_surfaces_2_2;
aircraft_surfaces_2_2_plot = rad2deg(aircraft_surfaces_2_2_plot);
PlotAircraftSim_5(t_2_2,aircraft_state_2_2,aircraft_surfaces_2_2_plot,fig_2_2,col,'2.2');


%% Part 3
pos_3 = [0, 0, -1800];
euler_3 = [15,-12,270 ]; % In degrees
euler_3 = deg2rad(euler_3); % Radians
v_body_3 = [19,3,-2];
rates_3 = [0.08,-0.2,0]; % degrees / second
rates_3 = deg2rad(rates_3); % rad/sec
aircraft_state_0_2_3 = [pos_3, euler_3, v_body_3, rates_3]';
aircraft_surfaces_2_3 = [5, 2, -13,0.3]'; % Degrees
aircraft_surfaces_2_3 = deg2rad(aircraft_surfaces_2_3); % Radians

t_span_2_3 = [0, 400];


[t_2_3,aircraft_state_2_3] = ode45(@(time,aircraft_state)AircraftEOM(time, aircraft_state, aircraft_surfaces_2_3,wind_inertial, aircraft_parameters),t_span_2_3,aircraft_state_0_2_3);

fig_2_3 = fig_2_2 + 10;
aircraft_surfaces_2_3_plot = ones(4,length(t_2_3));
aircraft_surfaces_2_3_plot = aircraft_surfaces_2_3_plot .* aircraft_surfaces_2_3;
aircraft_surfaces_2_3_plot = rad2deg(aircraft_surfaces_2_3_plot);
PlotAircraftSim_5(t_2_3,aircraft_state_2_3,aircraft_surfaces_2_3_plot,fig_2_3,col,'2.3');


%% Task 3
% Implementing the doublet effect
% Part 1
% Implementing doublet

doublet_time_1 = 0.25; % Seconcds
doublet_size_1 = 15; % Degrees
doublet_size_1 = deg2rad(doublet_size_1); % Rad

t_span_3_1 = [0,3];
% Execute the ODE solver with the doublet effect applied to the aircraft surfaces
[t_3_1, aircraft_state_3_1] = ode45(@(time, aircraft_state) AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces_2_2,doublet_size_1,doublet_time_1, wind_inertial, aircraft_parameters), t_span_3_1, aircraft_state_0_2_2);
% Backing out the elevator angles
aircraft_surfaces_3_1_plot = aircraft_surfaces_2_2.* ones(4,length(t_3_1));
for i = 1 : length(t_3_1)


if t_3_1(i) > 0 && t_3_1(i)<= doublet_time_1
    aircraft_surfaces_3_1_plot(1,i) = aircraft_surfaces_3_1_plot(1,i) + doublet_size_1; % First condition
elseif t_3_1(i) > doublet_time_1 && t_3_1(i)<= 2 * doublet_time_1
        aircraft_surfaces_3_1_plot(1,i) = aircraft_surfaces_3_1_plot(1,i) - doublet_size_1; % Second condition
     
end
end
aircraft_surfaces_3_1_plot = rad2deg(aircraft_surfaces_3_1_plot);

fig_3_1 = fig_2_1 + 100;


PlotAircraftSim_5(t_3_1,aircraft_state_3_1,aircraft_surfaces_3_1_plot,fig_3_1,col,'3.1');


%% Part 2
t_span_3_2 = [0,100];



% Execute the ODE solver with the doublet effect applied to the aircraft surfaces
[t_3_2, aircraft_state_3_2] = ode45(@(time, aircraft_state) AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces_2_2,doublet_size_1,doublet_time_1, wind_inertial, aircraft_parameters), t_span_3_2, aircraft_state_0_2_2);
% Backing out the elevator angles
aircraft_surfaces_3_2_plot = aircraft_surfaces_2_2.* ones(4,length(t_3_2));
for i = 1 : length(t_3_2)


if t_3_2(i) > 0 && t_3_2(i)<= doublet_time_1
    aircraft_surfaces_3_2_plot(1,i) = aircraft_surfaces_3_2_plot(1,i) + doublet_size_1; % First condition
elseif t_3_2(i) > doublet_time_1 && t_3_2(i)<= 2 * doublet_time_1
        aircraft_surfaces_3_2_plot(1,i) = aircraft_surfaces_3_2_plot(1,i) - doublet_size_1; % Second condition
     
end
end
aircraft_surfaces_3_2_plot = rad2deg(aircraft_surfaces_3_2_plot);

fig_3_2 = fig_3_1 + 10;


PlotAircraftSim_5(t_3_2,aircraft_state_3_2,aircraft_surfaces_3_2_plot,fig_3_2,col,'3.2');
