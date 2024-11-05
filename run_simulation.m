% MATLAB script to load data, run Simulink model, and plot results.

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

% Discrete methods
numStepsDiscrete = size(discreteData.x_HD, 2);
tDiscrete = linspace(0, t_span, numStepsDiscrete)'; % Generate time vector

% Continuous methods
numStepsContinuous = size(continuousData.x_HC, 2);
tContinuous = linspace(0, t_span, numStepsContinuous)';

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

%% Load Simulink Model
model = 'DronePIDModel';
load_system(model);

%% Simulation Configuration
methods = {'HD', 'TD', 'HC', 'TC'};
method_names = {'Discrete Heavy-Ball', 'Discrete Triple Momentum', 'Continuous Heavy-Ball', 'Continuous Triple Momentum'};
time_vectors = {tDiscrete, tDiscrete, tContinuous, tContinuous};
trajectories = {x_HD_discrete, x_TD_discrete, x_HC_continuous, x_TC_continuous};

% Initialize cell arrays to store simulation outputs
simOuts = cell(1, length(methods));
positions = cell(1, length(methods));

for i = 1:length(methods)
    try
        %% Prepare Reference Trajectory
        % Get trajectory data
        traj_data = trajectories{i};
        t_data = time_vectors{i};

        % Create timeseries objects
        ts_x_ref = timeseries(traj_data(:,1), t_data);
        ts_y_ref = timeseries(traj_data(:,2), t_data);

        % Assign to base workspace for Simulink
        assignin('base', 'ts_x_ref', ts_x_ref);
        assignin('base', 'ts_y_ref', ts_y_ref);

        % Set simulation stop time
        set_param(model, 'StopTime', num2str(t_data(end)));

        %% Run Simulation
        simOut = sim(model);

        % Store the simulation output
        simOuts{i} = simOut;

        %% Collect Outputs
        % Access the outputs from simOut
        try
            % Access x and y from simOut.x and simOut.y
            x_actual = simOut.x{1}.Values;
            y_actual = simOut.y{1}.Values;
        catch
            error('Could not access x and y outputs from simOut for method %s.', method_names{i});
        end

        % Combine x and y into a single matrix
        positions{i} = [x_actual.Data, y_actual.Data];

    catch ME
        fprintf('Error during simulation of method %s: %s\n', method_names{i}, ME.message);
        simOuts{i} = [];
        positions{i} = [];
    end
end

%% Plot Results
% Plot the trajectories from the simulations
figure;
hold on;
% Set axes limits and enable grid
xlim([-12, 12]);
ylim([-12, 12]);
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
title('Drone Trajectories from Simulink Simulations');
grid on;
set(gca, 'FontSize', 12);
hold off;

% Save the figure
exportgraphics(gcf, 'DroneTrajectories_Simulink.png', 'Resolution', 300);

%% Plot Position Errors Over Time
figure;
hold on;

for i = 1:length(methods)
    t_data = time_vectors{i};
    traj_data = trajectories{i};
    data = positions{i};

    if ~isempty(data)
        % Extract simulation time vector
        sim_time = x_actual.Time;
        x_data = x_actual.Data;
        y_data = y_actual.Data;
        
        % Interpolate reference trajectory onto sim_time
        x_ref_interp = interp1(t_data, traj_data(:,1), sim_time, 'linear', 'extrap');
        y_ref_interp = interp1(t_data, traj_data(:,2), sim_time, 'linear', 'extrap');
        
        % Compute position error
        position_error = sqrt((x_ref_interp - x_data).^2 + (y_ref_interp - y_data).^2);

        % Plot the position error
        plot(sim_time, position_error, 'Color', colors{i}, 'LineStyle', lineStyles{i}, 'LineWidth', lineWidths(i));
    end
end

xlabel('Time (s)');
ylabel('Position Error (m)');
title('Position Error Over Time');
legend(method_names, 'Location', 'Best');

% Enable grid
grid on;

set(gca, 'FontSize', 12);
hold off;

% Save the figure
exportgraphics(gcf, 'PositionError_Simulink.png', 'Resolution', 300);



% Enable grid
grid on;


% Save the figure
exportgraphics(gcf, 'PositionError_Simulink.png', 'Resolution', 300);



%% Plot Velocities Over Time to Show Smoothness
% Compute velocities
velocities = cell(1, length(methods));

for i = 1:length(methods)
    data = positions{i};
    t_data = time_vectors{i};

    if ~isempty(data)
        dt = diff(t_data);
        dx = diff(data(:,1));
        dy = diff(data(:,2));
        vx = dx ./ dt;
        vy = dy ./ dt;
        velocities{i} = [vx, vy];
    else
        velocities{i} = [];
    end
end

% Plot Speeds Over Time
figure;
hold on;

for i = 1:length(methods)
    t_data = time_vectors{i}(1:end-1); % Time vector for velocities

    vel_data = velocities{i};

    if ~isempty(vel_data)
        speed = sqrt(vel_data(:,1).^2 + vel_data(:,2).^2);

        plot(t_data, speed, 'Color', colors{i}, 'LineStyle', lineStyles{i}, 'LineWidth', lineWidths(i));
    end
end

xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Speed Over Time');
legend(method_names, 'Location', 'Best');
grid on;
set(gca, 'FontSize', 12);
hold off;

% Save the figure
exportgraphics(gcf, 'SpeedOverTime_Simulink.png', 'Resolution', 300);

%% Close the Simulink model
close_system(model, 0); % The 0 means don't save changes
