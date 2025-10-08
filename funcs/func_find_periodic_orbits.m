function periodic_orbits = func_find_periodic_orbits(states, epsilon, T_max, dt)
    % FUNC_FIND_PERIODIC_ORBITS - Detects periodic orbits in a dynamical system
    % Inputs:
    %   states   - NxD matrix of system states
    %   epsilon  - Tolerance for return (normalized space)
    %   T_max    - Max time (not step) to check for periodicity
    %   dt       - Time step size

    % Output:
    %   periodic_orbits - Struct array with fields: period, orbit_points

    % Normalize each dimension to [0, 1]
    min_vals = min(states);
    max_vals = max(states);
    norm_states = (states - min_vals) ./ (max_vals - min_vals + eps);

    time_steps = size(states, 1);
    max_cycle = floor(T_max / dt);

    periodic_orbits = struct('period', {}, 'orbit_points', {});
    
    for n = 1:(time_steps - max_cycle)
        ref_state = norm_states(n, :);
        found = false;

        for T = 50:max_cycle
            if n + T > time_steps
                break;
            end

            candidate_state = norm_states(n + T, :);
            dist_return = norm(candidate_state - ref_state);

            if dist_return < epsilon
                orbit_segment = states(n : n + T - 1, :);  % unnormalized
                period_time = T * dt;
                periodic_orbits(end+1).period = period_time;
                periodic_orbits(end).orbit_points = orbit_segment;
                found = true;
                break;  % Stop after first valid detection

            end
        end
    end
end


