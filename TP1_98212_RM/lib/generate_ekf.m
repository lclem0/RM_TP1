function [xstate_true,control_input_true,control_input_mea,obs_range_bearing,landmarkxy,obs_landmark_ID, rows_with_nan, num_rows_with_nan]=generate_ekf(N,sig_v,sig_omega,sig_r,sig_phi,beacon_coords,velocities,true_points, obsNoise)

    % fuction adapted by Joao Clemente (98212) from the course's lectures

    landmarkxy = beacon_coords;
    control_input_true = velocities(:,1:2);
    num_steps = size(control_input_true, 1);

    %% Add noise to control inputs
    control_input_mea = control_input_true;
    noises_v = randn(num_steps, 1) * sig_v;
    noises_omega = randn(num_steps, 1) * sig_omega;
    control_input_mea(:, 1) = control_input_mea(:, 1) + noises_v;
    control_input_mea(:, 2) = control_input_mea(:, 2) + noises_omega;

    %% Generate true robot poses
    xstate_true = true_points; % Assuming all_points includes initial pose at 0th index

    %% Generate observation data
    obs_range_bearing = [];
    for i = 1:num_steps
        current_pose = true_points(i,:);
        detected_beacons = BeaconDetection(N, current_pose, obsNoise); % Assuming obs_noise is handled internally

        obs_vector = i;
        for j = 1:length(detected_beacons)
            noise_r = randn * sig_r;
            noise_phi = randn * sig_phi;
            noisy_d = detected_beacons(j).d + noise_r;
            noisy_a = detected_beacons(j).a + noise_phi;

            obs_vector = [obs_vector, j, noisy_d, noisy_a]; % Assuming j as ID for simplicity
        end
        obs_range_bearing = [obs_range_bearing; obs_vector];
    end


    % Filter out NaN bearings
    rows_with_nan = any(isnan(obs_range_bearing(:, 4:3:end)), 2); % Check every third column starting from the fourth
    num_rows_with_nan = sum(rows_with_nan);



    %% Randomize observation sequence for each time step (optional)
    obs_landmark_ID = [(0:num_steps-1)', zeros(num_steps, N)];
    for i = 1:num_steps
        obs_landmark_ID(i, 2:end) = randperm(N);
    end
end

