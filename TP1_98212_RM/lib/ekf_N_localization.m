function [xstate_EKF, P_EKF] = ekf_N_localization(control_input_mea, obs_range_bearing, sig_v, sig_omega, sig_r, sig_phi, landmarkxy, Dt, xstate_true)

    % fuction adapted by Joao Clemente (98212) from the course's lectures



    % Number of steps in the simulation
    num_steps = size(control_input_mea, 1); 

    % Number of landmarks
    N = size(landmarkxy, 1);

    % Define the noise covariances for the EKF
    Q = [sig_v^2 0; 0 sig_omega^2];
    R_i = [sig_r^2 0; 0 sig_phi^2];

    % Initialize the state and covariance matrices
    xstate_EKF = zeros(num_steps+1, 3); % Added one more for the initial state
    P_EKF = repmat(0.01 * eye(3), 1, 1, num_steps+1); % 3D array to hold each step's covariance matrix

    % Loop over each time step
    for step = 1:num_steps
        fprintf('Running step %d...\n', step);

        % Current state and covariance
        xstate_t = xstate_EKF(step, :);
        P_t = P_EKF(:, :, step);

        % Current control input
        control_t = control_input_mea(step, :);

        % Observation data at current step
        obs_t1 = obs_range_bearing(step, 2:end);

        % Check for NaN in observation data to decide whether to skip
        if any(isnan(obs_t1))
            fprintf('Skipping step %d due to NaN in observation data.\n', step);
            xstate_EKF(step + 1, :) = xstate_t; % Carry forward the last state
            P_EKF(:, :, step + 1) = P_t; % Carry forward the last covariance
            continue; % Skip this iteration
        end

        % Assemble the observation noise covariance matrix for all landmarks
        R = kron(eye(N), R_i); % This creates a block diagonal matrix with R_i repeated N times

        % Run the Extended Kalman Filter step
        [xstate_t1, P_t1] = ekf_N_landmarks(xstate_t, P_t, control_t, obs_t1, landmarkxy, Dt, Q, R, N);

        % Store the updated state and covariance
        xstate_EKF(step + 1, :) = xstate_t1;
        P_EKF(:, :, step + 1) = P_t1;
    end

     % Plot the results
    figure(1);
    hold on;
    grid on;
    plot(landmarkxy(:, 1), landmarkxy(:, 2), 'k*', 'MarkerSize', 14);
    text(landmarkxy(:, 1) + 0.2, landmarkxy(:, 2), num2str((1:N)'), 'FontWeight', 'bold', 'FontSize', 14);

    % Plot each state with uncertainty ellipse
    for i = 1:num_steps+1
        plot(xstate_EKF(i, 1), xstate_EKF(i, 2), 'bo', 'LineWidth', 3);
        quiver(xstate_EKF(i, 1), xstate_EKF(i, 2), cos(xstate_EKF(i, 3)), sin(xstate_EKF(i, 3)), 0.5, 'Color', 'b', 'LineWidth', 2);
        
        % Additionally plot the true trajectory for comparison
        plot(xstate_true(i, 1), xstate_true(i, 2), 'ro', 'LineWidth', 2);
        quiver(xstate_true(i, 1), xstate_true(i, 2), cos(xstate_true(i, 3)), sin(xstate_true(i, 3)), 0.5, 'Color', 'r', 'LineWidth', 1.5);
    end

    legend('Landmarks', 'EKF Estimated Position', 'EKF Estimated Orientation', 'True Position', 'True Orientation', 'Location', 'best');
    xlabel('X Position (meters)');
    ylabel('Y Position (meters)');
    title('Extended Kalman Filter Localization Results');
end
