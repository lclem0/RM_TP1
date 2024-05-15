function [xstate_t1, P_t1] = ekf_N_landmarks(xstate_t, P_t, control_t, obs_t1, landmarkxy, Dt, Q, R, N)
    disp('------------------------------------------------');
    disp('Running Extended Kalman Filter');

    % fuction adapted by Joao Clemente (98212) from the course's lectures
    
    % Prediction step
    pzero =[0 0];  % set noise equal to 0 for the prediction
    [xstatet1_t] = motionmodel(xstate_t, control_t, pzero, Dt);

    % Jacobian of the state transition
    temp = -Dt * control_t(1) * sin(xstate_t(3));
    temp2 = Dt * control_t(1) * cos(xstate_t(3));
    Jfx = [1 0 temp;
           0 1 temp2;
           0 0 1];

    % Jacobian of the process noise
    temp3 = Dt * control_t(1) * cos(xstate_t(3));
    temp4 = Dt * control_t(1) * sin(xstate_t(3));
    Jfw = [temp3 0;
           temp4 0;
           0 Dt];

    Pt1_t = Jfx * P_t * Jfx' + Jfw * Q * Jfw';

    % Update step
    z_all = [];
    z_pred = [];
    Jh = [];

    for i = 1:N
        id = obs_t1(3 * i - 2);
        landmark_i = landmarkxy(id, :);
        z_i = obs_t1(3 * i - 1 : 3 * i);
        z_pred_i = sensormodel(landmark_i, xstatet1_t, [0 0]);
        z_all = [z_all; z_i'];
        z_pred = [z_pred; z_pred_i'];
        Jh_i = jacobi(landmark_i, xstatet1_t(1), xstatet1_t(2));
        Jh = [Jh; Jh_i];
    end

    innov = z_all - z_pred;
    for i = 1:size(innov, 1)
        innov(i, 1) = wrap(innov(i, 1));
    end

    S = Jh * Pt1_t * Jh' + R;
    K = Pt1_t * Jh' / S;

    xstatet1_t1 = xstatet1_t' + K * innov;
    Pt1_t1 = Pt1_t - K * Jh * Pt1_t;


    xstate_t1 = xstatet1_t1';
    P_t1 = Pt1_t1;
end

function nu = wrap(alpha)
    while alpha > pi
        alpha = alpha - 2 * pi;
    end
    while alpha < -pi
        alpha = alpha + 2 * pi;
    end
    nu = alpha;
end

