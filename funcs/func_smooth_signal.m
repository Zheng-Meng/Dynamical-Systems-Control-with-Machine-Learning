function signal_smooth = func_smooth_signal(N, signal_range,key_interval, cutoff_freq, dt)
    % FUNC_SMOOTH_SIGMA - Generates smoothly varying sigma values over time
    % Inputs:
    %   N           - Number of time steps
    %   signal_range - Range of control values [min, max]
    %   dt          - Time step

    % Output:
    %   sigma_smooth - Smoothed sigma values

    % key_interval = 0.5; % Change signal every 1 second
    key_points = 0:key_interval/dt:N;  
    sigma_key = signal_range(1) + (signal_range(2) - signal_range(1)) * rand(length(key_points), 1);

    % Interpolate signal values smoothly over time
    signal_interp = interp1(key_points, sigma_key, 1:N, 'pchip');

    % Apply a low-pass filter to remove high-frequency noise
    % cutoff_freq = 2; % Cutoff at 1 Hz (smooth changes over 1 second)
    sampling_rate = 1/dt;
    [b, a] = butter(2, cutoff_freq / (sampling_rate / 2)); % Butterworth low-pass filter
    signal_smooth = filtfilt(b, a, signal_interp); % Apply zero-phase filtering

    signal_smooth = signal_smooth';
end
