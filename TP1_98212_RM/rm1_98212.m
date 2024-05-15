%% tp1 - 98212 
function rm1_98212(N,Dt,r,L,Vn,Wn,V)

addpath lib\
close all;clc;


if nargin<7
    V = 5; %desired average linear velocity along trajectory (m/s)
end
if nargin<6
    Wn=0.1; % angular velocity uncertainty
end
if nargin<5
    Vn=0.1; % velocity uncertainty
end
if nargin<4
    L=1; % wheel base
end
if nargin < 3
    r=0.15; %m wheel radius
end
if nargin < 2
    Dt = 1; %time step
end
if nargin < 1
    N = 4; %number of beacons
end

%Valores Iniciais
P = [0,0,0];
obsNoise= [0.25, 0.1];
% V = 5; %m/s
%generate beacons
B = BeaconDetection(N,P,obsNoise);

beacon_coords = []; % Initialize variable to store beacon coordinates

for i = 1:length(B)
    % Concatenate x and y coordinates into a single variable
    beacon_coords = [beacon_coords; B(i).X, B(i).Y];
end

% total_x_y is not the pchip true values.
% true_points is the true values from pchip
[total_x, total_y, true_points] = plotPath(beacon_coords, V, Dt);

%%
% Calculate linear and angular velocities
[linear_velocities, angular_velocities, linear_velocities_noise, angular_velocities_noise] = calculateVelocities(true_points, Dt, Vn, Wn);
% Concatenate all velocities into one matrix
velocities = [linear_velocities angular_velocities linear_velocities_noise angular_velocities_noise];
%plot velocities function
plot_velocities(velocities)

% ate aqui está tudo certo
%% noise level setting -- to generating data (must be the same when running the EKF)
%control: velocity, turnrate
sig_v     = 0.1;
sig_omega = 0.1;

% observation: range, bearing
sig_r   = 0.1;
sig_phi = 0.1;

% landmark data -- landmarkxy
% control data -- control_input_mea
% observation data -- obs_range_bearing
% true robot pose data -- xstate_true (for comparison)
[xstate_true,control_input_true,control_input_mea,obs_range_bearing,landmarkxy,obs_landmark_ID, rows_with_nan, num_rows_with_nan]=generate_ekf(N,sig_v,sig_omega,sig_r,sig_phi,beacon_coords,velocities,true_points, obsNoise);

%% ekf localization

% ekf_N_localization(control_input_mea, obs_range_bearing,sig_v,sig_omega,sig_r,sig_phi, xstate_true, landmarkxy, Dt)
[xstate_EKF, P_EKF] = ekf_N_localization(control_input_mea, obs_range_bearing, sig_v, sig_omega, sig_r, sig_phi, landmarkxy, Dt, xstate_true);
        

%% velocidades para DD e TRI


%% save into file
[N, ~] = size(xstate_EKF);  
filename = 'loc_98212.txt';
fileID = fopen(filename, 'w');

for i = 1:N
    rowStr = sprintf('%f,%f,%f\n', xstate_EKF(i, :));    
    fprintf(fileID, rowStr);
end

fclose(fileID);

end