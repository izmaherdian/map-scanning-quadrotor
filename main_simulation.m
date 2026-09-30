%% MAIN_SIMULATION  Nonlinear backstepping control of a quadrotor for zig-zag map scanning.
% Plain-text version of main_simulation.mlx (for reading on GitHub / running without the Live Editor).
% Initializes the quadrotor parameters and controller gains, runs model/QuadrotorModel.slx
% and prints the RMSE of the position and attitude tracking.

%% Quadrotor Parameters Initialization
clear
clc
close all

% Add project folders (Simulink model, helper scripts, data) to the path
projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot, 'model'), fullfile(projectRoot, 'scripts'), fullfile(projectRoot, 'data'));

%% Parameter
% Inertia
Ixx = 7.5e-3;
Iyy = 7.5e-3;
Izz = 1.3e-2;

% Axel Length
l = 0.23;

% Rotor Inertia
Jr = 6e-5;

% Mass
m = 0.650;
g = 9.81;

% Aerodynamic Force and Moments Constant
kf = 3.13e-5;
km = 7.5e-7;

% % Aerodynamic Coefficients
% Krx = 0; % 0.1;
% Kry = 0; % 0.1;
% Krz = 0; % 0.15;
% Ktx = 0; % 0.1;
% Kty = 0; % 0.1;
% Ktz = 0; % 0.15;

% Constants Calculations
a1 = (Iyy - Izz)/Ixx;
a2 = Jr/Ixx;
a3 = (Izz - Ixx)/Iyy;
a4 = Jr/Iyy;
a5 = (Ixx - Iyy)/Izz;
b1 = 1/Ixx;
b2 = 1/Iyy;
b3 = 1/Izz;

% Rotor Dynamics
% R_mot = 0.6; % Motor Circuit Resistance
% K_mot = 5.2; % Motor Torque Constant 

% Disturbances
noise_rot = 0;
noise_trans = 0;

% Desired Velocities and Accelerations
z_dot_d = -0.46;
phi_dot_d = 0;
theta_dot_d = 0;
psi_dot_d = 0;

z_dot_dot_d = 0;
phi_dot_dot_d = 0;
theta_dot_dot_d = 0;
psi_dot_dot_d = 0;

% Hover Omega
omega_hover = sqrt((m*g)/(4*kf));
deltaU1_max = kf*4*omega_hover^2;
deltaU2_max = kf*omega_hover^2;
deltaU4_max = km*2*omega_hover^2;

omega_max = (90*100)*9000*(2*pi/(60)); %90% of 9000 rpm to rad/s
U1_max = kf*4*omega_max^2;
U2_max = kf*omega_max^2;
U4_max = km*2*omega_max^2;

% Rotor Dynamic
numerator = 0.936; 
denominator = [0.178 1]; 

[A, B, C, D] = tf2ss(numerator, denominator);

%% Desired Positions
% % Desired Positions
% phi_d = deg2rad(10);        % Desired Roll Angle
% theta_d = deg2rad(0);       % Desired Pitch Angle
% psi_d = deg2rad(0);         % Desired Yaw Angle
% z_d = -20;                  % Desired Altitude (in meters)

%% Tuning Params Backstepping Control
% Assign optimal parameters to base workspace
assignin('base', 'c1', 2);
assignin('base', 'c2', 1);
assignin('base', 'c3', 2);
assignin('base', 'c4', 1);
assignin('base', 'c5', 2);
assignin('base', 'c6', 1);
assignin('base', 'c7', 0.308);
assignin('base', 'c8', 3);

% Assign params for position controller
assignin('base', 'k1', 0.5)
assignin('base', 'k2', 0.5)
assignin('base', 'k3', 1)
assignin('base', 'k4', 1)

%% Lintasan Drone
panjang = 50;               % Panjang lintasan x
lebar = 55;                 % Lebar area pada sumbu y
belokan = 10;               % Jarak antar lintasan y
kecepatan = 5;              % Kecepatan drone
waktu_simulasi = 120;       % Durasi pause untuk animasi

%% Jalankan Simulink
% Running
% load('DataLintasanDroneFix(ScanningMode).mat');
simOut = sim('QuadrotorModel',waktu_simulasi);

xData = simOut.x_out;
yData = simOut.y_out;
zData = simOut.z_out;

phiData = simOut.phi_out;
thetaData = simOut.theta_out;
psiData = simOut.psi_out;

tData = simOut.t_out;

xdotData = simOut.xdot_out;
ydotData = simOut.ydot_out;
zdotData = simOut.zdot_out;

phidotData = simOut.phidot_out;
thetadotData = simOut.thetadot_out;
psidotData = simOut.psidot_out;

x = xData.Data(:,1);
x_d = xData.Data(:,2);
y = yData.Data(:,1);
y_d = yData.Data(:,2);
z = zData.Data(:,1);
z_d = zData.Data(:,2);
XYZs = [x y z];
XYZd = [x_d y_d z_d];

phi = phiData.Data(:,1);
phi_d = phiData.Data(:,2);
theta = thetaData.Data(:,1);
theta_d = thetaData.Data(:,2);
psi = psiData.Data(:,1);
psi_d = psiData.Data(:,2);
EulerAngles = [phi theta psi];
EulerAngled = [phi_d theta_d psi_d];

t = tData.Data(:,1);

xdot = xdotData.Data(:,1);
ydot = ydotData.Data(:,1);
zdot = zdotData.Data(:,1);

phidot = phidotData.Data(:,1);
thetadot = thetadotData.Data(:,1);
psidot = psidotData.Data(:,1);

UData = simOut.U_out;
U1 = UData.Data(:,1);
U2 = UData.Data(:,2);
U3 = UData.Data(:,3);
U4 = UData.Data(:,4);

%% Root Mean Squared Error
% Hitung RMSE untuk posisi (x, y, z)
rms_pos = sqrt(mean((XYZs - XYZd).^2, 1)); % RMSE untuk masing-masing dimensi posisi
rms_x = rms_pos(1); % RMSE untuk x
rms_y = rms_pos(2); % RMSE untuk y
rms_z = rms_pos(3); % RMSE untuk z

% Hitung RMSE untuk sudut Euler (phi, theta, psi)
rms_euler = sqrt(mean((EulerAngles - EulerAngled).^2, 1)); % RMSE untuk masing-masing sudut
rms_phi = rms_euler(1); % RMSE untuk phi
rms_theta = rms_euler(2); % RMSE untuk theta
rms_psi = rms_euler(3); % RMSE untuk psi

% Tampilkan hasil
disp('RMSE untuk posisi:');
disp(['x: ', num2str(rms_x)]);
disp(['y: ', num2str(rms_y)]);
disp(['z: ', num2str(rms_z)]);

disp('RMSE untuk sudut Euler:');
disp(['phi: ', num2str(rms_phi)]);
disp(['theta: ', num2str(rms_theta)]);
disp(['psi: ', num2str(rms_psi)]);
