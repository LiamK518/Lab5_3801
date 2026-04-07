function var_dot = AircraftEOM(time, aircraft_state, aircraft_surfaces,wind_inertial, aircraft_parameters)


x = aircraft_state(1,1); % Inertial X Position (m)
y = aircraft_state(2,1); % Inertial Y Position (m)
z = aircraft_state(3,1); % Inertial Z Position (m)
psi = aircraft_state(6,1); % Body Yaw angle (rad)
theta = aircraft_state(5,1); % Body Pitch Angle (rad)
phi = aircraft_state(4,1); % Body Roll Angle (rad)
u_e = aircraft_state(7,1); % X velcocity in Body Coordinates (m/s)
v_e = aircraft_state(8,1); % Y velocity in Body Coordinates (m/s)
w_e = aircraft_state(9,1); % Z velocity in Body Coordinates (m/s)
p = aircraft_state(10,1); % Body Roll rate (rad/s)
q = aircraft_state(11,1); % Body Pitch rate (rad/s)
r = aircraft_state(12,1); % Body Yaw rate (rad/s)


% Gravity
g = aircraft_parameters.g; 


Ix = aircraft_parameters.Ix;
Iy = aircraft_parameters.Iy;
Iz = aircraft_parameters.Iz;
Ixz = aircraft_parameters.Ixz;









cpsi = cos(psi);  spsi = sin(psi);
cth  = cos(theta); sth  = sin(theta);
cphi = cos(phi);  sphi = sin(phi);

% derivitave velocity 
x_dot = [ cth.*cpsi,  sphi.*sth.*cpsi - cphi.*spsi,  cphi.*sth.*cpsi + sphi.*spsi ] ;
y_dot = [ cth.*spsi,  sphi.*sth.*spsi + cphi.*cpsi,  cphi.*sth.*spsi - sphi.*cpsi ] ;
z_dot = [ -sth, sphi.*cth,  cphi.*cth ] ;
v_body = [u_e; v_e; w_e];

% Inertial position rates
pos_dot = [x_dot; y_dot; z_dot] * v_body;

xE_dot = pos_dot(1);
yE_dot = pos_dot(2);
zE_dot = pos_dot(3);

% Euler Angle Rates
phi_dot = (p + (sin(phi) .* tan(theta) .* q) + (cos(phi) .* tan(theta) .* r));
theta_dot = ((cos(phi) .* q) + (-sin(phi) .* r));
psi_dot = ((sin(phi) .* sec(theta)) .* q + (cos(phi) .* sec(theta) .* r));





% Calculating the Aero Froces and Moments
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
% Pulling out Forces
X = aero_forces(1);
Y = aero_forces(2);
Z = aero_forces(3);

% Pulling out Moments
L = aero_moments(1); % Roll moment
M = aero_moments(2); % Pitch Moment
N = aero_moments(3); % Yaw Moment



% Velocity Rates
u_e_dot = ((r .* v_e - q .* w_e) + (g .* -sin(theta)) + (X ./ m));
v_e_dot = ((p .* w_e - r .* u_e) + (g .* cos(theta) .* sin(phi)) + (Y ./ m));
w_e_dot = ((q .* u_e - p .* v_e) + (g .* cos(theta) .* cos(phi)) + (Z ./ m));



%% --------- EDIT THIS --------- %%

% Making gammas 
gamma = Ix * Iz - Ixz ^2;
gamma_1 = Ixz * (Ix- Iy + Iz) / gamma;
gamma_2 = (Iz(Iz-Iy) + Ixz^2) / gamma;
gamma_3 = Iz / gamma;
gamma_4 = Ixz / gamma;
gamma_5 = (Iz-Ix) / gamma;
gamma_6 = Ixz / Iy;
gamma_7 = (Ix(Ix-Iy)+Ixz^2)/gamma;
gamma_8 = Ix / gamma;
% Angular Velocity Rates
p_dot = (gamma_1 * p * q - gamma_2 * q * r) + (gamma_3 * L + gamma_4 * N);
q_dot = (gamma_5 * p * r - gamma_6(p^2 - r^2)) + (1/Iy) * M;
r_dot = (gamma_7 * p * q - gamma_1 * 1 * r) + (gamma_4 * L + gamma_8 * N);



var_dot = [xE_dot yE_dot zE_dot phi_dot theta_dot psi_dot u_e_dot v_e_dot w_e_dot p_dot q_dot r_dot]';




end




































