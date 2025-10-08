function dxdt = func_lorenz(state, control_signal)
    % LORENZ_EQ - Computes the derivatives of the Lorenz system.
    % Inputs:
    %   state - [x, y, z] state vector (1x3)

    % Output:
    %   dxdt - Derivatives [dx/dt, dy/dt, dz/dt] (1x3)

    x = state(1);
    y = state(2);
    z = state(3);

    sigma = 10;
    rho = 28;
    beta = 8/3;
    
    u1 = control_signal(1);
    u2 = control_signal(2);
    u3 = control_signal(3);

    dxdt = [
        sigma * (y - x) + u1;
        x * (rho - z) - y + u2;
        x * y - beta * z + u3
    ]';
end
