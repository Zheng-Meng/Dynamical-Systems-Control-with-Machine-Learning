clear all
close all
clc

addpath('funcs/')

% Load trained RC model
load('save_rc/trained_rc_lorenz_control.mat', 'Wout', 'r_end', 'A', 'W_in', 'alpha', 'output_dims');

% Load detected periodic orbits
load('save_data/lorenz_periodic_orbits.mat');

% Simulation parameters
T_control = 5000; % Control for long duration
dt = 0.02;       % Time step

u1_range = [-10, 10];
u2_range = [-10, 10];
u3_range = [-20, 20];

param_range = [u1_range; u2_range; u3_range];

% Select periodic orbit to follow 
target_index = 400;
selected_orbit = periodic_orbits(target_index).orbit_points;

% Initialize Lorenz state
current_state = [1, 1, 1];

% Find the closest initial target point on the periodic orbit
[~, closest_idx] = min(vecnorm(selected_orbit - current_state, 2, 2));

% Store controlled trajectory & control signals
controlled_trajectory = zeros(T_control, 3);
control_params = zeros(T_control-1, length(output_dims)); % Stores varying parameter values
controlled_trajectory(1, :) = current_state(:)'; % Ensure row vector

% Use the stored periodic orbit directly
% Extend periodic orbit to match control duration
num_repeats = ceil(T_control / size(selected_orbit, 1)); % How many times to repeat
ref_orbit_extended = repmat(selected_orbit, num_repeats, 1); % Repeat periodic orbit
ref_orbit = ref_orbit_extended(1:end, :); % Trim to exact length

% Control Loop
for t = 2:T_control
    % Step 1: Determine the target state from the periodic orbit
    target_state = ref_orbit(t, :); % Correctly sized

    % Step 2: Predict the control signals using the trained RC model
    predicted_signals = func_rc_predict(Wout, r_end, current_state, target_state, W_in, A, alpha, param_range);

    % Step 4: Simulate one step of the controlled Lorenz system
    current_state = func_rk4_step(@func_lorenz, current_state, predicted_signals, dt);
    % current_state = current_state';  % Ensure row vector format

    % Store results
    controlled_trajectory(t, :) = current_state(:);
    control_params(t-1, :) = predicted_signals;
end

%% 
time = (0:T_control-1) * dt; % Correctly define the time vector

% ----------------------------------------
% PLOT 1: Time Series (Controlled vs. Reference)
% ----------------------------------------
figure;
subplot(3,1,1);
plot(time, controlled_trajectory(:,1), 'r--', 'LineWidth', 1.2);
hold on;
plot(time, ref_orbit(1:T_control,1), 'b-', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('X');
title('Controlled vs. Reference Orbit (X)');
legend('Controlled', 'Reference');
grid on;

subplot(3,1,2);
plot(time, controlled_trajectory(:,2), 'r--', 'LineWidth', 1.2);
hold on;
plot(time, ref_orbit(1:T_control,2), 'b-', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Y');
title('Controlled vs. Reference Orbit (Y)');
legend('Controlled', 'Reference');
grid on;

subplot(3,1,3);
plot(time, controlled_trajectory(:,3), 'r--', 'LineWidth', 1.2);
hold on;
plot(time, ref_orbit(1:T_control,3), 'b-', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Z');
title('Controlled vs. Reference Orbit (Z)');
legend('Controlled', 'Reference');
grid on;

% ----------------------------------------
% PLOT 2: Control Parameters Over Time
% ----------------------------------------
time_control = (0:T_control-2) * dt; % Time vector for control parameters
num_params = size(control_params, 2);

figure('Position', [100, 100, 800, 600]);

for i = 1:num_params
    subplot(num_params, 1, i);
    plot(time_control, control_params(:, i), 'LineWidth', 1.2);
    xlabel('Time (s)');
    ylabel(sprintf('Param %d', i));
    title(sprintf('Control Parameter %d Over Time', i));
    grid on;
end

sgtitle('Control Parameters Over Time');


% ----------------------------------------
% PLOT 3: 3D Controlled vs. Reference Orbit
% ----------------------------------------
figure;
hold on;
plot3(ref_orbit(1:T_control,1), ref_orbit(1:T_control,2), ref_orbit(1:T_control,3), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Reference Orbit');
plot3(controlled_trajectory(:,1), controlled_trajectory(:,2), controlled_trajectory(:,3), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Controlled Orbit');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Controlled vs. Reference Orbit');
legend('show');
grid on;
axis equal;
view(45, 30); % Adjust viewpoint for better visualization
hold off;

% ----------------------------------------
% PLOT 4: 3D Controlled vs. Reference Orbit
% ----------------------------------------
start_time = 500;
figure;
hold on;
plot3(ref_orbit(start_time:T_control,1), ref_orbit(start_time:T_control,2), ref_orbit(start_time:T_control,3), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Reference Orbit');
plot3(controlled_trajectory(start_time:end,1), controlled_trajectory(start_time:end,2), controlled_trajectory(start_time:end,3), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Controlled Orbit');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Controlled vs. Reference Orbit');
legend('show');
grid on;
axis equal;
view(45, 30); % Adjust viewpoint for better visualization
hold off;
