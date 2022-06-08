% Generate MPC Constraints
clc; clear; close all
addpath(genpath([pwd '/Functions']))

%% Time Parameters / Obstacle
% Time Variables
Ts = 0.05;         % Computer Update rate (operating time)
Tfinal = 15;
r_min = 0.1;
r_min2 = 0.5;

%% Create Figure and Axes Handles:
f = figure();
f.Position = [10 10 1280 720]; 
ax1 = axes(f);
ax2 = axes(f);
ax3 = axes(f);

% Create Subplot Axes:
ax1 = subplot(4, 5, [1, 2, 3, 6, 7, 8, 11, 12, 13, 16, 17, 18], ax1);
ax2 = subplot(4, 5, [4, 5, 9, 10], ax2);
ax3 = subplot(4, 5, [14, 15, 19, 20], ax3);

% Create Plots:
p1_cell = cell(5, 1);
axes(ax1);
hold on;
transform_object = hgtransform('Parent', ax1);
drone = scatter(0, 0, 'o', 'MarkerFaceColor', '#6495ED', 'MarkerEdgeColor', 'none');
drone_trajectory = plot(0, 0, 'LineStyle', '--', 'Color', '#565656');
obstacle = scatter(0, 0, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'none');
obstacle_trajectory = scatter(0, 0, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.1);
obstacle_radius = patch('XData', r_min2.*cos(linspace(0, 2*pi)), 'YData', r_min2.*sin(linspace(0, 2*pi)), 'FaceColor', 'none', 'EdgeColor', '#a9a9a9');
constraint_line = fplot(0, 'Color', 'r');
risk_projection = quiver(0, 0, 1, 1, 'r');
hold off;

set(obstacle_radius, 'Parent', transform_object);

title(ax1, 'Simulation:', 'interpreter', 'latex')
xlabel(ax1, '$X$ (m)', 'interpreter', 'latex')
ylabel(ax1, '$Y$ (m)', 'interpreter', 'latex')
grid(ax1, 'minor')

p1_cell{1} = drone;
p1_cell{2} = drone_trajectory;
p1_cell{3} = obstacle;
p1_cell{4} = obstacle_trajectory;
p1_cell{5} = constraint_line;
p1_cell{6} = risk_projection;
p1_cell{7} = transform_object;

p2_cell = cell(3, 1);
axes(ax2)
hold on;
sample_plot = scatter(0, 0, 'o', 'MarkerFaceColor', '#a9a9a9', 'MarkerEdgeColor', 'none');
spline_points = scatter(0, 0, 'o', 'MarkerFaceColor', '#6495ED', 'MarkerEdgeColor', 'none');
spline = plot(0, 0, 'Color', '#6495ED');
hold off;

title(ax2,  'Failure Probability', 'interpreter', 'latex');
xlabel(ax2, '$\delta$', 'interpreter', 'latex')
ylabel(ax2, '$r(\delta)$', 'interpreter', 'latex')
grid(ax2, 'minor')

p2_cell{1} = sample_plot;
p2_cell{2} = spline_points;
p2_cell{3} = spline;

p3_cell = cell(3, 1);
axes(ax3)
hold on;
logsurvival = plot(0, 0, 'Color', '#6495ED');
logsurvival_points = scatter(0, 0, 'o', 'MarkerFaceColor', '#6495ED', 'MarkerEdgeColor', 'none');
logsurvival_eval = scatter(0, 0, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'none');
logsurvival_optim_eval = scatter(0, 0, 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'none');
hold off;

title(ax3,  'Log-Survival', 'interpreter', 'latex');
xlabel(ax3, '$\delta$', 'interpreter', 'latex')
ylabel(ax3, '$\tilde{S}$', 'interpreter', 'latex')
grid(ax3, 'minor')

p3_cell{1} = logsurvival;
p3_cell{2} = logsurvival_points;
p3_cell{3} = logsurvival_eval;
p3_cell{4} = logsurvival_optim_eval;

plot_cell = cell(3, 1);
plot_cell{1} = p1_cell;
plot_cell{2} = p2_cell;
plot_cell{3} = p3_cell;

data = cell(13, 1);

% GIF:
Frame(1) = getframe(gcf);
frame_iter = 1;

%% Setup the Problem Objectives and Constraints
Time = 0; Time2 = 0; i = 1;
[Th,Nodes,xd_lb,xd_ub,Numb_Obst,Dims,Ad,Bd,spline_resolution] = DevMPCRisk();

%% Drone Information
% Desired State - [x; y; z; dx; dy; dz], [x1, x2, ... xn]
qd_i = [(rand(Dims,1)-0.5).*xd_lb*2; zeros(Dims,1)];
qd_des = [(rand(Dims,1)-0.5).*xd_lb*2; zeros(Dims,1)];

% Initial Trajectory
qd = qd_i.*ones(size(qd_i,1),Nodes); % Q Desired
qd_prev = qd_i.*ones(size(qd_i,1),Nodes);

% Initial Risk Function:
m = zeros(spline_resolution, 1);
b = zeros(spline_resolution, 1);

% Logger and other counters:
risk_cell = cell(1, 1);
failureprobability_cell = cell(1, 1);
logsurvival_cell = cell(1, 1);
constraint_cell = cell(1, 1);
drone_cell = cell(1, 1);
trajectory_cell = cell(1, 1);
adversary_cell = cell(1, 1);
halfspace_cell = cell(1, 1);
risk_logger = [];
failure_counter = 0;

%% Obstacle Information
% Randomized Gains for Chasing Drone
% Kp = (rand(1,Numb_Obst)) + 0.3;
% Kd = (rand(1,Numb_Obst)).*.2 + 0.1;
Kp = 5*0.3;
Kd = 5*0.1;

% Initial States - [x; y; z; dx; dy; dz]
qo_i = [(rand(Dims,Numb_Obst)-0.5).*4; zeros(Dims,Numb_Obst)];

% Create the Obstacle State Space Model
co = .5;
Ao = zeros(Dims*2);
Ao(1:Dims,Dims+1:end) = eye(Dims);
Ao(Dims+1:end,Dims+1:end) = -co*eye(Dims);

Bo = zeros(Dims*2,Dims);
Bo(Dims+1:end,1:Dims) = eye(Dims);

%% Simulate the results
% Saving the Data
folderName = 'Test';
if ~exist(folderName, 'dir')
    mkdir([pwd '/', folderName])
end
addpath(genpath([pwd '/', folderName]))

% [s.x, s.y, s.z] = sphere(10);
% if Dims == 3
%     zlim([-3,3]);
%     view(23,45);
% end

% Initial Data Log:
% Risk Data:
risk_cell{1} = risk_logger;
failureprobability_cell{1} = [0, 0];
logsurvival_cell{1} = [0, 0];
constraint_cell{1} = [m, b];

% Trajectory Data:
trajectory_cell{1} = [qd; zeros(2, Nodes)];
drone_cell{1} = qd_i.';
adversary_cell{1} = qo_i.';

% Half-Space Data:
[projected_distance, n_vector, predicted_drone_states, predicted_adversary_states] = RiskSourceFormulation(qd_i, qd, qd_prev, qo_i, Th, Nodes);
% halfspace_line = @(x) -n_vector(1, :) .* (x -  predicted_adversary_states(1, :)) ./ n_vector(2, :) + predicted_adversary_states(2, :);
func_cell = cell(Nodes, 1);
for j = 1:Nodes
    f = @(x) -n_vector(1, j) .* (x -  predicted_adversary_states(1, j)) ./ n_vector(2, j) + predicted_adversary_states(2, j);
    func_cell{j} = f;
end
halfspace_line_all = @(x) -n_vector(1, :) .* (x -  predicted_adversary_states(1, :)) ./ n_vector(2, :) + predicted_adversary_states(2, :);
halfspace_cell{1, 1} = projected_distance;
halfspace_cell{1, 2} = n_vector;
halfspace_cell{1, 3} = func_cell;
halfspace_cell{1, 4} = predicted_drone_states;
halfspace_cell{1, 5} = predicted_adversary_states;

% Set x0:
x0 = qd_i;

while Time < Tfinal
    % Save the Results
%     save([folderName, '\Iter_' num2str(i)], 'qd', 'qd_i', 'qd_des', 'r_min', 'r_min2', 'Th', 'Nodes')
    
    % Create a random control input for the object
    x_e = qd_i - qo_i;
    uo = Kp.*x_e(1:Dims,:) + Kd.*x_e(Dims+1:end,:);
    
    % Develop the desired Trajectory
    [qd, ud, fval, design_vector] = RunMPCRisk(Th, Nodes, qd_i, qo_i, qd_des, qd_prev, xd_lb, xd_ub, m, b, x0);
    [delta, ~, ~, ~] = RiskSourceFormulation(qd_i, qd, qd_prev, qo_i, Th, Nodes);
    if ~isempty(design_vector)
        x0 = design_vector;
        percieved_risk = design_vector(end-Nodes+1:end);
    else
        percieved_risk = zeros(Nodes, 1);
    end
    
    % Measure the Actual Motion
    [Tsim, qd_sim, qo_sim, simulation_step] = Simulate(Th,Ts,Nodes,qd_i,qo_i,ud,uo,Ad,Bd,Ao,Bo);
    
    % Update the states, time and iteration
    qd_i = qd_sim(end,:).';
    for j = 0:Numb_Obst-1
        qo_i(:,j+1) = qo_sim(end,j*Dims*2+1:(j+1)*Dims*2).';
    end
    qd_prev = qd;
    
    % Risk Learning and Simulation:
    % SIMULATION ONLY:
    [risksample_x, risksample_y, failure_flag] = riskEvaluation(qd_sim, qo_sim, r_min2);
    failure_counter = failure_counter + failure_flag;
    % Log risk sample:
    risk_logger = [risk_logger; risksample_x, risksample_y];
    % Failure Probability Spline:
    [fpf_x, fpf_y] = riskcurveRegression(risk_logger(:, 1), risk_logger(:, 2), spline_resolution);
    % Find Weights of Fit:
    weight_vector = binFunction(risk_logger(:, 1), fpf_x, spline_resolution);
    % Log-Survival Spline:
    [ls_x, ls_y] = logsurvivalRegression(fpf_x, fpf_y, weight_vector);
    % Update Constraint: (Wait for First Failure)
    if failure_counter > 1
        [m, b] = lineEquation(ls_x, ls_y);
    end
    
    % Update the states, time and iteration
    Time = Time + Ts;
    i = i + 1;
    
    % Log Data:
    % Risk Data:
    risk_cell{i} = risk_logger;
    failureprobability_cell{i} = [fpf_x.', fpf_y];
    logsurvival_cell{i} = [ls_x.', ls_y];
    constraint_cell{i} = [m, b];
    
    % Trajectory Data:
    trajectory_cell{i} = [qd; ud];
    drone_cell{i} = qd_sim;
    adversary_cell{i} = qo_sim;
    
    % Half-Space Data:
    [projected_distance, n_vector, predicted_drone_states, predicted_adversary_states] = RiskSourceFormulation(qd_i, qd, qd_prev, qo_i, Th, Nodes);
%     halfspace_line = @(x) -n_vector(1, :) .* (x -  predicted_adversary_states(1, :)) ./ n_vector(2, :) + predicted_adversary_states(2, :);
    func_cell = cell(Nodes, 1);
    for j = 1:Nodes
        f = @(x) -n_vector(1, j) .* (x -  predicted_adversary_states(1, j)) ./ n_vector(2, j) + predicted_adversary_states(2, j);
        func_cell{j} = f;
    end
    halfspace_cell{i, 1} = projected_distance;
    halfspace_cell{i, 2} = n_vector;
    halfspace_cell{i, 3} = func_cell;
    halfspace_cell{i, 4} = predicted_drone_states;
    halfspace_cell{i, 5} = predicted_adversary_states;
    
    % Plot Data:
    %Plot 1:
    data{1} = drone_cell{i};
    data{2} = trajectory_cell{i};
    data{3} = adversary_cell{i};
    data{4} = predicted_adversary_states;
    data{5} = func_cell;
    data{9} = percieved_risk;
    data{10} = n_vector;
    data{11} = projected_distance;
    data{12} = risksample_x;
    %Plot 2:
    data{6} = risk_cell{i};
    data{7} = failureprobability_cell{i};
    %Plot 3:
    data{8} = logsurvival_cell{i};
    data{13} = delta;
    
    % Animate:
    [plot_cell, Frame, frame_iter] = animate(f, [ax1, ax2, ax3], plot_cell, data, Frame, frame_iter);
    
    % Restart Environment if Failure Occurs:
    if failure_flag == 1
        % Reset Agent:
        qd_i = [(rand(Dims,1)-0.5).*xd_lb*2; zeros(Dims,1)];
        qd = qd_i.*ones(size(qd_i,1), Nodes);
        qd_prev = qd_i.*ones(size(qd_i,1),Nodes);
        % Reset Adversary:
        qo_i = [(rand(Dims,Numb_Obst)-0.5).*4; zeros(Dims,Numb_Obst)];
        failure_flag = 0;
    end
end

%% Video and Animation Functions:
% create the video writer
name = 'simulation_results_';
% writerObj = VideoWriter(name,'Motion JPEG AVI');
writerObj = VideoWriter(name,'MPEG-4');
writerObj.FrameRate = 33;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:floor(numel(Frame(1,:))/2)
    % convert the image to a frame
    frame = Frame(i) ;
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);

save('data.mat', 'risk_cell', 'failureprobability_cell', 'logsurvival_cell', 'constraint_cell', 'trajectory_cell', 'drone_cell', 'adversary_cell', 'halfspace_cell')