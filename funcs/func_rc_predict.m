function predicted_params = func_rc_predict(Wout, r_end, x, x_target, W_in, A, alpha, param_range)
    % FUNC_RC_PREDICT - Predicts the varying parameter(s) given the current and target Lorenz state
    % Inputs:
    %   Wout        - Trained output weights
    %   r_end       - Final reservoir state
    %   x           - Current Lorenz state [x, y, z]
    %   x_target    - Target state on periodic orbit [x_target, y_target, z_target]
    %   W_in        - Input weight matrix
    %   A           - Reservoir matrix
    %   alpha       - Leak rate
    %   param_range - [min_value, max_value] range applied to all predicted parameters

    % Output:
    %   predicted_params - Predicted control parameter(s), clamped within the given range

    % Convert input to feature vector
    u = [x(:); x_target(:)];  % Concatenate current and target states

    % Update reservoir state
    r_new = (1 - alpha) * r_end + alpha * tanh(A * r_end + W_in * u);

    % Predict control parameters
    predicted_params = Wout * r_new;

    % Clamp predictions to the given range
    predicted_params(1) = max(param_range(1,1), min(param_range(1,2), predicted_params(1)));
    predicted_params(2) = max(param_range(2,1), min(param_range(2,2), predicted_params(2)));
    predicted_params(3) = max(param_range(3,1), min(param_range(3,2), predicted_params(3)));
end

