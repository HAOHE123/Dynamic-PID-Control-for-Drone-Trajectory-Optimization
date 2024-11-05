% MATLAB script to simulate drone trajectories using PID controller with dynamic gain adjustment.

% Clear workspace and close figures
clear; close all; clc;

%% Load Data
% Load GMM data
GMMData = load('GMM_Model.mat');

% Load discrete optimization data
discreteData = load('discrete_data.mat');

% Load continuous optimization data
continuousData = load('continuous_data.mat');

%% Prepare Time Vectors
t_span = 5; % Total time span defined in Python

% Simulation time settings
dt_sim = 0.01; % Simulation time step
t_sim = 0:dt_sim:t_span; % Simulation time vector

%% Prepare Trajectory Data
% Discrete methods
x_HD_discrete = discreteData.x_HD';
x_TD_discrete = discreteData.x_TD';

% Continuous methods
x_HC_continuous = continuousData.x_HC';
x_TC_continuous = continuousData.x_TC';

% Optimal point
x0_mean = GMMData.means(2,:);

% GMM settings
means = GMMData.means;
covariances = GMMData.covariances;

%% Simulation Configuration
methods = {'HD', 'TD', 'HC', 'TC'};
method_names = {'Discrete Heavy-Ball', 'Discrete Triple Momentum', 'Continuous Heavy-Ball', 'Continuous Triple Momentum'};

% Original time vectors for trajectories
tDiscrete = linspace(0, t_span, size(x_HD_discrete, 1))';
tContinuous = linspace(0, t_span, size(x_HC_continuous, 1))';

% Trajectories and time vectors
trajectories = {x_HD_discrete, x_TD_discrete, x_HC_continuous, x_TC_continuous};
t_orig_vectors = {tDiscrete, tDiscrete, tContinuous, tContinuous};

% Initialize cell arrays to store simulation outputs
positions = cell(1, length(methods));
velocities = cell(1, length(methods));

%% PID Controller Parameters
% Initial PID gains (you may need to tune these)
Kp = 5;
Ki = 0.1;
Kd = 0.5;

for i = 1:length(methods)
    %% Prepare Reference Trajectory
    traj_data = trajectories{i};
    t_orig = t_orig_vectors{i};
    
    % Interpolate or hold reference trajectory based on method type
    if i <= 2 % Discrete methods
        % Create step-wise reference trajectory
        x_ref = interp1(t_orig, traj_data(:,1), t_sim, 'previous', 'extrap');
        y_ref = interp1(t_orig, traj_data(:,2), t_sim, 'previous', 'extrap');
    else % Continuous methods
        % Interpolate reference trajectory smoothly
        x_ref = interp1(t_orig, traj_data(:,1), t_sim, 'linear', 'extrap');
        y_ref = interp1(t_orig, traj_data(:,2), t_sim, 'linear', 'extrap');
    end
    
    %% Initialize Variables for Simulation
    num_steps = length(t_sim);
    x_actual = zeros(num_steps, 2); % [x, y]
    v_actual = zeros(num_steps, 2); % [vx, vy]
    error_integral = zeros(1, 2);   % Integral of error [ex_i, ey_i]
    error_previous = zeros(1, 2);   % Previous error [ex_prev, ey_prev]
    
    % Initial positions (assuming starting from the first point of the trajectory)
    x_actual(1, :) = traj_data(1, :);
    v_actual(1, :) = [0, 0]; % Assuming starting from rest
    
    for k = 1:num_steps - 1
        % Reference position at current time
        x_ref_k = [x_ref(k), y_ref(k)];
        
        % Compute error between reference and actual position
        error = x_ref_k - x_actual(k, :);
        
        % Update integral of error
        error_integral = error_integral + error * dt_sim;
        
        % Compute derivative of error
        if k == 1
            error_derivative = [0, 0];
            delta_speed = 0;
        else
            error_derivative = (error - error_previous) / dt_sim;
            % Compute change in speed
            speed_current = norm(v_actual(k, :));
            speed_previous = norm(v_actual(k - 1, :));
            delta_speed = speed_current - speed_previous;
        end
        
        % Adjust PID gains if velocity is decreasing
        if delta_speed < 0
            % Velocity is decreasing
            % Adjust PID gains to slow down more rapidly
            Kp_current = Kp * 0.5; % Reduce Kp
            Ki_current = Ki * 0.5; % Reduce Ki
            Kd_current = Kd * 10.0; % Increase Kd
        else
            % Use standard gains
            Kp_current = Kp;
            Ki_current = Ki;
            Kd_current = Kd;
        end
        
        % PID control law for acceleration (assuming mass = 1 for simplicity)
        acceleration = Kp_current * error + Ki_current * error_integral + Kd_current * error_derivative;
        
        % Update velocities
        v_actual(k + 1, :) = v_actual(k, :) + acceleration * dt_sim;
        
        % Update positions
        x_actual(k + 1, :) = x_actual(k, :) + v_actual(k + 1, :) * dt_sim;
        
        % Update previous error
        error_previous = error;
    end
    
    % Store the simulation output
    positions{i} = x_actual;
    velocities{i} = v_actual;
end

%% Plot Results
% Plot the trajectories from the simulations
figure;
hold on;
% Set axes limits and enable grid
xlim([-12, 12]);
ylim([-12, 12]);
grid on;

colors = {'b', 'r', 'g', 'k'};
lineStyles = {'-', '-', '--', '--'}; % Solid for discrete, dashed for continuous methods
lineWidths = [1.5, 1.5, 2, 2]; % Thicker lines for continuous methods

for i = 1:length(methods)
    data = positions{i};
    if ~isempty(data)
        plot(data(:,1), data(:,2), 'Color', colors{i}, 'LineStyle', lineStyles{i}, 'LineWidth', lineWidths(i));
    end
end

% Plot the GMM means
plot(means(:,1), means(:,2), 'mo', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', 'c', 'DisplayName', 'GMM Means');

% Plot the optimal point
plot(x0_mean(1), x0_mean(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'Optimal Point');

legend([method_names, {'GMM Means', 'Optimal Point'}], 'Location', 'Best');

xlabel('X-axis');
ylabel('Y-axis');
title('Drone Trajectories with PID Controller and Dynamic Gain Adjustment');
set(gca, 'FontSize', 12);
hold off;

% Save the figure
exportgraphics(gcf, 'DroneTrajectories_PID_DynamicGain.png', 'Resolution', 300);

%% Plot Position Errors Over Time
figure;
hold on;
grid on;

for i = 1:length(methods)
    t_data = t_sim;
    data = positions{i};
    
    if ~isempty(data)
        % Reference trajectory
        if i <= 2 % Discrete methods
            x_ref = interp1(t_orig_vectors{i}, trajectories{i}(:,1), t_sim, 'previous', 'extrap');
            y_ref = interp1(t_orig_vectors{i}, trajectories{i}(:,2), t_sim, 'previous', 'extrap');
        else % Continuous methods
            x_ref = interp1(t_orig_vectors{i}, trajectories{i}(:,1), t_sim, 'linear', 'extrap');
            y_ref = interp1(t_orig_vectors{i}, trajectories{i}(:,2), t_sim, 'linear', 'extrap');
        end
        
        % Compute position error
        position_error = sqrt((x_ref - data(:,1)).^2 + (y_ref - data(:,2)).^2);
        
        % Plot the position error
        plot(t_data, position_error, 'Color', colors{i}, 'LineStyle', lineStyles{i}, 'LineWidth', lineWidths(i));
    end
end

xlabel('Time (s)');
ylabel('Position Error (m)');
title('Position Error Over Time with Dynamic Gain Adjustment');
legend(method_names, 'Location', 'Best');
set(gca, 'FontSize', 12);
hold off;


%% Plot Velocities Over Time to Show Smoothness
figure;
hold on;
grid on;

for i = 1:length(methods)
    t_data = t_sim;
    v_data = velocities{i};
    
    if ~isempty(v_data)
        speed = sqrt(v_data(:,1).^2 + v_data(:,2).^2);
    
        % Plot the speed
        plot(t_data, speed, 'Color', colors{i}, 'LineStyle', lineStyles{i}, 'LineWidth', lineWidths(i));
    end
end

xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Speed Over Time with Dynamic Gain Adjustment');
legend(method_names, 'Location', 'Best');
set(gca, 'FontSize', 12);
hold off;

% Save the figure
exportgraphics(gcf, 'SpeedOverTime_PID_DynamicGain.png', 'Resolution', 300);
