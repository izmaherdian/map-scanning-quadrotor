%% GA_TUNE_ALTITUDE  Tune the altitude backstepping gains (c7, c8) with a Genetic Algorithm.
% Requires the Global Optimization Toolbox. Run main_simulation first so that
% the quadrotor parameters are in the base workspace and model/ is on the path.

%% GA Parameter Tuning
% Jumlah parameter yang dioptimasi (c1, c2)
nvars = 2;

% Batas bawah dan atas parameter untuk c1 dan c2
lb = [0.1, 0.1]; % Lower bounds
ub = [10, 10];   % Upper bounds

% Opsi Genetic Algorithm
options = optimoptions('ga', ...
    'PopulationSize', 20, ...
    'MaxGenerations', 50, ...
    'Display', 'iter', ...
    'UseParallel', false); % Aktifkan parallel computing jika tersedia

% Jalankan GA
[optimal_params, optimal_fitness] = ga(@performanceBackstepping, nvars, [], [], [], [], lb, ub, [], options);

% Assign optimal parameters (c1, c2) ke base workspace
assignin('base', 'c1', 1);
assignin('base', 'c2', 1);
assignin('base', 'c3', 1); % Nilai tetap
assignin('base', 'c4', 1); % Nilai tetap
assignin('base', 'c5', 1); % Nilai tetap
assignin('base', 'c6', 1); % Nilai tetap
assignin('base', 'c7', optimal_params(1)); % Nilai tetap
assignin('base', 'c8', optimal_params(2)); % Nilai tetap

%% GA Obcjective Function
function perf = performanceBackstepping(k)
    % Assign Backstepping parameters ke base workspace
    assignin('base', 'c1', 1);
    assignin('base', 'c2', 1);
    assignin('base', 'c3', 1); % Nilai tetap
    assignin('base', 'c4', 1); % Nilai tetap
    assignin('base', 'c5', 1); % Nilai tetap
    assignin('base', 'c6', 1); % Nilai tetap
    assignin('base', 'c7', k(1)); % Nilai tetap
    assignin('base', 'c8', k(2)); % Nilai tetap

    % Simulasikan model Simulink
    simOut = sim('QuadrotorModel', 'StopTime', '10', ...
                 'SaveOutput', 'on', ...
                 'SimulationMode', 'normal');

    % Ekstrak respons ketinggian (z) dan waktu
    zData = simOut.get('yout');                 % Altitude response
    zElement = zData.getElement('altitude');
    zValues = zElement.Values.Data;
    tValues = zElement.Values.Time;
    z = zValues(:);
    t = tValues(:);

    % Hitung metrik performa
    S = stepinfo(z, t, 'SettlingTimeThreshold', 0.02);
    settling_time = S.SettlingTime;                     % Settling time
    overshoot = S.Overshoot;                            % Overshoot
    steady_state_error = abs(z(end) - 1);               % Target altitude = 1

    % Fungsi fitness (minimalkan settling time, overshoot, dan steady-state error)
    perf = settling_time + 10 * overshoot + 100 * steady_state_error;
end
