function [linear_velocities, angular_velocities, linear_velocities_noise, angular_velocities_noise] = calculateVelocities(true_points, Dt, Vn, Wn)
   
    % function created by Joao Clemente (98212) to obtain the velocities

    positions = true_points(:, 1:2); % x, y positions
    angles = true_points(:, 3); % direction angles

    linear_velocities = zeros(size(positions, 1)-1, 1);
    linear_velocities_noise = zeros(size(positions, 1)-1, 1);
    angular_velocities = zeros(size(angles, 1)-1, 1);
    angular_velocities_noise= zeros(size(angles, 1)-1, 1);

    for i = 1:length(true_points)-1
        % Calculate linear velocity
        distance = sqrt((positions(i+1,1) - positions(i,1))^2 + (positions(i+1,2) - positions(i,2))^2);
        linear_velocities(i) = distance / Dt;

        % Calculate angular velocity
        angle_diff = angles(i+1) - angles(i);
        angular_velocities(i) = angle_diff / Dt;

        % Apply noise
        linear_velocities_noise(i) = linear_velocities(i) + Vn * randn();
        angular_velocities_noise(i) = angular_velocities(i) + Wn * randn();
    end

    
end
