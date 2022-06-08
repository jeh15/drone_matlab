clc; clear; close all;
%% Load Data
load('data.mat');

%% Trajectory Data:
% MPC Trajectory Cell: Trajectory Provided by Optimization
% trajectory_cell - Data Format - [x; y; dx; dy; ux; uy] - Row-wise

% Drone Trajectory Cell: Drone Trajectory from Simulation
% drone_cell - Data Format - [x, y, dx, dy] - Columnwise

% Adversary Trajectory Cell: Adversary Trajectory from Simulation
% adversary_cell - Data Format - [x, y, dx, dy] - Columnwise

%% Risk Data:
% Simulated Risk Sample Cell:
% risk_cell - Data Format - [risk source sample, evaluated sample] - Columnwise

% Failure Probability Spline Cell:
% failureprobability_cell - Data Format - [x, y] - Columnwise

% Log-Survival Spline Cell:
% logsurvival_cell - Data Format - [x, y] - Columnwise

% Constraint Lines Cell: (rfun = m * delta + b)
% constraint_cell - Data Format - [m, b] - Columnwise

%% Halfspace Data:
% Halfspace Cell: halfspace_cell
% Data Format - Columnwise
%{
halfspace_cell{MPC_Iteration, 1} = linearized distances
halfspace_cell{MPC_Iteration, 2} = normal vectors - [x; y] - Row-wise
halfspace_cell{MPC_Iteration, 3} = cell containing halfspace constraints
halfspace_cell{MPC_Iteration, 4} = predicted drone trajectory - [x; y] - Row-wise
halfspace_cell{MPC_Iteration, 5} = predicted adversary trajectory - [x; y] - Row-wise
%}

% Halfspace constraint line function cell: halfspace_cell{:, 3}
%{
This cell provides an anonymous function that represents the halfspace
constraint at every node along the trajectory.

Ex. halfspace_cell{2, 3}{1} = First Halfspace Constraint for the second MPC iteration 
%}
%%