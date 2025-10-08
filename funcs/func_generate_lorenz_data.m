function training_data = func_generate_lorenz_data(T, dt, control_signals, initial_state)
    % GENERATE_LORENZ_DATA - Simulates Lorenz system with time-varying parameters.
    % Inputs:
    %   T              - Total simulation time
    %   dt             - Time step
    %   params         - Nx3 matrix of time-varying Lorenz parameters [sigma, rho, beta]
    %   initial_state  - Initial [x, y, z] state

    % Output:
    %   training_data  - Struct containing states and corresponding parameters

    % Time vector
    t_span = 0:dt:T;
    N = length(t_span);
    
    % Initialize trajectory storage
    states = zeros(N, 3);
    states(1, :) = initial_state;

    % Simulate the Lorenz system using RK4
    for i = 1:N-1
        current_state = states(i, :);
        current_signal = control_signals(i, :);  % Time-varying parameters
        states(i+1, :) = func_rk4_step(@func_lorenz, current_state, current_signal, dt);
    end

    % Store results
    training_data = struct('states', states(1:end-1, :), ...
                           'next_states', states(2:end, :), ...
                           'control_signals', control_signals(1:end-1, :));
end
