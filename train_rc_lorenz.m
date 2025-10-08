clear all
close all
clc

%%
addpath('funcs')

% Simulation parameters
T = 500;       % Total time
dt = 0.02;    % Time step
initial_state = [1, 1, 1];  % Initial condition
N = T / dt;   % Number of time steps

% Smoothing settings
key_interval = 0.5; % Interval for generating key parameter values
cutoff_freq = 2;    % Cutoff frequency for filtering

% control signal range
u1_range = [-10, 10];
u2_range = [-10, 10];
u3_range = [-20, 20];

u1_values = func_smooth_signal(N, u1_range, key_interval, cutoff_freq, dt);
u2_values = func_smooth_signal(N, u2_range, key_interval, cutoff_freq, dt);
u3_values = func_smooth_signal(N, u3_range, key_interval, cutoff_freq, dt);

control_signals = [u1_values, u2_values, u3_values];

figure();
plot(control_signals(1:1000, 1));

% Run Lorenz system simulation
training_data = func_generate_lorenz_data(T, dt, control_signals, initial_state);

%% Reservoir Computing Training
train_length = floor(0.7 * N);  % 70% for training
val_length = floor(0.2 * N);    % 20% for validation

% Define hyperparameters
n = 500;          % Reservoir size
beta = 1e-6;      % Ridge regression regularization
alpha = 0.3;      % Leak rate
k = 0.1;          % Sparsity of reservoir connections
eig_rho = 0.95;   % Spectral radius
W_in_a = 0.1;     % Input scaling
noise_a = 1e-3;   % Noise level

% Select output dimensions based on which parameters vary
output_dims = [1, 2, 3];

% Train reservoir computing
[rmse, Wout, r_end, A, W_in, val_real, val_pred] = func_rc_train_val(...
    training_data, train_length, val_length, n, beta, alpha, k, eig_rho, W_in_a, noise_a, output_dims);

% Save trained model
save('./save_rc/trained_rc_lorenz_control.mat', 'Wout', 'r_end', 'A', 'W_in', 'alpha', 'val_real', 'val_pred', 'output_dims');

%% Plot validation predictions vs. real values
figure;
t_span = linspace(0, dt * val_length, val_length);

for i = 1:length(output_dims)
    subplot(length(output_dims), 1, i);
    plot(t_span(100:end), val_real(i, 100:end), 'k', 'LineWidth', 1.5); hold on;
    plot(t_span(100:end), val_pred(i, 100:end), 'r--', 'LineWidth', 1.5);
    ylabel(['Param ', num2str(output_dims(i))]);
    legend('True', 'Predicted');
    grid on;
end
xlabel('Time');
sgtitle('Reservoir Computing Validation: True vs. Predicted Parameters');

disp('Training complete! RC model saved.'); 














