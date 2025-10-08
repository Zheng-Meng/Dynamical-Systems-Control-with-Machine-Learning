clear all
close all
clc

% Simulate Lorenz system
T = 1000;
dt = 0.02;
initial_state = [1,1,1];
N = T / dt;   % Number of time steps

u1_values = zeros(N,1);
u2_values = zeros(N,1);
u3_values = zeros(N,1);
control_signals = [u1_values, u2_values, u3_values]; % set no control signals

training_data = func_generate_lorenz_data(T, dt, control_signals, initial_state);
states = training_data.states; % Extract trajectory

% Find periodic orbits
epsilon = 0.05;  % Tolerance
T_max = 2;     % Maximum period length
periodic_orbits = func_find_periodic_orbits(states, epsilon, T_max, dt);

% ---------------- Sort by period ----------------
if ~isempty(periodic_orbits)
    [~, order] = sort([periodic_orbits.period], 'descend');
    periodic_orbits = periodic_orbits(order);
end

% --- Save periodic orbits ---
save('save_data/lorenz_periodic_orbits.mat', 'periodic_orbits');

% --- Print Found Periods ---
disp('Detected periodic orbits:');
for i = 1:length(periodic_orbits)
    fprintf('Period: %d\n', periodic_orbits(i).period);
end

%%
figure; hold on;
attractor_length = min(10000, size(states,1));
plot3(states(1:attractor_length,1), states(1:attractor_length,2), states(1:attractor_length,3), ...
      'k', 'LineWidth', 0.5, 'DisplayName', 'Attractor');

target_index = 400;
if length(periodic_orbits) >= target_index
    po = periodic_orbits(target_index);
    plot3(po.orbit_points(:,1), po.orbit_points(:,2), po.orbit_points(:,3), ...
          'LineWidth', 3, 'DisplayName', sprintf('Sorted #%d, T=%.6f', target_index, po.period));
else
    warning('Only %d periodic orbits found; cannot plot the 10th.', length(periodic_orbits));
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('Periodic Orbit (10th in sorted list) within Attractor');
view(3); grid on; legend; hold off;



