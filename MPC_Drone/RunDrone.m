% Generate MPC Constraints
clc; clear; close all
addpath(genpath([pwd '/Functions']))

%% Time Parameters
% Time Variables
Ts = 0.05;         % Computer Update rate (operating time)
Tfinal = 30;

%% Setup the Problem Objectives and Constraints
Time = 0; Time2 = 0; i = 1;
[Th,Nodes,xd_lb,xd_ub,Numb_Obst,Dims,Ad,Bd,spline_resolution] = DevMPC();

r_min = 0.1;
r_min2 = 0.5;
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
risk_logger = [];
failure_counter = 0;

%% Obstacle Information
Kp = 0.3;
Kd = 0.1;

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

% Figure Information
h1 = figure();
filename = 'Test.gif';
figure(h1)

[s.x, s.y, s.z] = sphere(10);
if Dims == 3
    zlim([-3,3]);
    view(23,45);
end

while Time < Tfinal
    % Save the Results
%     save([folderName, '\Iter_' num2str(i)], 'qd', 'qd_i', 'qd_des', 'r_min', 'r_min2', 'Th', 'Nodes')
    
    % Animate
    Animation(i, s, Dims, Numb_Obst, h1, Ts, r_min, r_min2, qo_i, qd_i, qd, qd_des, [folderName,'/',filename])
    
    % Create a random control input for the object
    x_e = qd_i - qo_i;
    uo = Kp.*x_e(1:Dims,:) + Kd.*x_e(Dims+1:end,:);
    
    % Develop the desired Trajectory
    [qd, ud, fval, design_vector] = RunMPC(Th, Nodes, qd_i, qo_i, qd_des, qd_prev, xd_lb, xd_ub, m, b);
    disp('Objective Function Value:')
    disp(fval)
    disp('Log-Survival Slack Value:')
    disp(sum(design_vector(end-Nodes:end)));
    
    % Measure the Actual Motion
    [Tsim, qd_sim, qo_sim] = Simulate(Th,Ts,Nodes,qd_i,qo_i,ud,uo,Ad,Bd,Ao,Bo);
    
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
    k = 1;
    weight_vector = zeros(spline_resolution+1, 1);
    for j = 1:numel(risk_logger(:, 1))
        if risk_logger(j, 1) <= fpf_x(k+1)
            weight_vector(k) = weight_vector(k) + 1;
            weight_vector(k+1) = weight_vector(k+1) + 1;
        else
            k = k + 1;
        end
    end
    % Log-Survival Spline:
    [ls_x, ls_y] = logsurvivalRegression(fpf_x, fpf_y, weight_vector);
    % Update Constraint: (Wait for First Failure)
    if failure_counter > 1
        [m, b] = lineEquation(ls_x, ls_y);
    end
    
    % Plot Risk Constraints for DEBUGGING:
%     p1.XData = fpf_x;
%     p1.YData = fpf_y;
%     p2.XData = ls_x;
%     p2.YData = ls_y;
    
    % Update the states, time and iteration
    Time = Time + Ts;
    Time2 = Time2 + Ts;
    i = i + 1;
    
    % Restart Environment if Failure Occurs:
    if failure_flag == 1
        % Reset Agent:
        qd_i = [(rand(Dims,1)-0.5).*xd_lb*2; zeros(Dims,1)];
        qd = qd_i.*ones(size(qd_i,1), Nodes);
        qd_prev = qd_i.*ones(size(qd_i,1),Nodes);
        % Reset Adversary:
        qo_i = [(rand(Dims,Numb_Obst)-0.5).*4; zeros(Dims,Numb_Obst)];
    end
end
