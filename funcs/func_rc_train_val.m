function [rmse, Wout, r_end, A, W_in, val_real, val_pred] = func_rc_train_val(training_data, train_length, val_length, n, beta, alpha, k, eig_rho, W_in_a, noise_a, output_dims)
    % TRAIN_RC_LORENZ - Generalized reservoir computing training for Lorenz system
    % Inputs:
    %   training_data - Preloaded dataset (continuous time series)
    %   train_length  - Number of training samples (continuous segment)
    %   val_length    - Number of validation samples (next segment)
    %   output_dims   - Indices of params to predict (e.g., [1] for sigma, [1,2] for sigma & rho in lorenz system)

    % Outputs:
    %   rmse   - Root Mean Squared Error of validation
    %   Wout   - Output weight matrix
    %   r_end  - Final reservoir state for later use
    %   A      - Reservoir weight matrix (needed for testing)
    %   W_in   - Input weight matrix (needed for testing)
    %   val_real - Validation true values
    %   val_pred - Validation predicted values

    % Extract continuous training segment
    start_idx = 1; % Start training from the beginning of the dataset
    train_idx = start_idx:(start_idx + train_length - 1);
    val_idx = (start_idx + train_length):(start_idx + train_length + val_length - 1);

    % Inputs: [x_n, y_n, z_n, x_n+1, y_n+1, z_n+1]
    train_x = [training_data.states(train_idx, :), training_data.next_states(train_idx, :)]';
    val_x = [training_data.states(val_idx, :), training_data.next_states(val_idx, :)]';

    % Extract the selected dimensions from params
    train_y = training_data.control_signals(train_idx, output_dims)';
    val_y = training_data.control_signals(val_idx, output_dims)';

    % Initialize reservoir
    W_in = W_in_a * (2 * rand(n, size(train_x, 1)) - 1);
    A = sprandsym(n, k);
    eig_D = eigs(A, 1);
    A = (eig_rho / abs(eig_D)) * A;
    A = full(A);

    % Training phase
    r_all = zeros(n, train_length + 1);
    train_x = train_x + noise_a * randn(size(train_x)); % Add noise

    for ti = 1:train_length
        r_all(:, ti+1) = (1 - alpha) * r_all(:, ti) + alpha * tanh(A * r_all(:, ti) + W_in * train_x(:, ti));
    end

    % Train Wout using ridge regression
    Wout = (train_y * r_all(:, 2:end)') / (r_all(:, 2:end) * r_all(:, 2:end)' + beta * eye(n));

    % Store final reservoir state
    r_end = r_all(:, end);

    % Validation Phase
    r_val = zeros(n, val_length + 1);
    val_pred = zeros(size(val_y));

    for t = 1:val_length
        r_val(:, t+1) = (1 - alpha) * r_val(:, t) + alpha * tanh(A * r_val(:, t) + W_in * val_x(:, t));
        val_pred(:, t) = Wout * r_val(:, t+1);
    end

    % Compute RMSE on validation set
    error = val_pred - val_y;
    rmse = sqrt(mean(error.^2, 'all'));

    % Return validation real vs predicted for plotting
    val_real = val_y;
end
