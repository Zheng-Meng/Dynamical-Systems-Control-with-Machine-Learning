function next_state = func_rk4_step(f, state, params, dt)
    % func_rk4_step - One step RK4 integration with params
    % Inputs:
    %   state  - Current state [x, y, z]
    %   params - System parameters
    %   dt     - Time step
    % Output:
    %   next_state - Next state after RK4 integration

    % Compute RK4 steps
    k1 = dt * f(state, params);
    k2 = dt * f(state + k1 / 2, params);
    k3 = dt * f(state + k2 / 2, params);
    k4 = dt * f(state + k3, params);

    % Compute next state
    next_state = state + (k1 + 2*k2 + 2*k3 + k4) / 6;
end