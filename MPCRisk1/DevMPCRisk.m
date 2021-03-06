function [Th,Nodes,xd_lb,xd_ub,Numb_Obst,dims_x,Ad,Bd,spline_resolution] = DevMPCRisk()

% Generate MPC Constraints
% clc; clear; close all
addpath(genpath([pwd '/MPCRisk/Functions']))

%% Input Parameters
% Time Variables
Th = 2;          % Time Horizon (lookahead time)

% Problem Nodes
Nodes = 21;

% Number of Obstacles
Numb_Obst = 1;

% Spline Resolution:
spline_resolution = 4;

% Upper/Lower Bounds - [x; y; z; Fx; Fy; Fz], [max, min]
% size of this determines the dimensions
xd_lb = [-5; -5];
xd_ub = [5; 5];  

% Velocity Upper/Lower Bounds - [dx; dy; dz], [max, min]
vd_lb = [-100; -100];
vd_ub = [100; 100];

% Control Input Upper/Lower Bounds
ud_lb = [-10; -10];
ud_ub = [10; 10];

% Cost Function Values
% Design Vector - 
dims_x = size(xd_lb,1);
dims_u = size(vd_lb,1);

% Cost Function Values
W = [0.0; 0.001; 1.0];

% Create the Drone State Space Model 
c = -0.1;
Ad = zeros(dims_x*2);
Ad(1:dims_x,dims_x+1:end) = eye(dims_x);
Ad(dims_x+1:end,dims_x+1:end) = eye(dims_x).*c;
   
Bd = zeros(dims_x*2,dims_u);
Bd(dims_x+1:end,1:dims_x) = eye(dims_x);

%% Setup the Problem Objectives and Constraints
% Develop the Constraint Functions
[x,u,p,dv] = Constraints(Th,Nodes,Numb_Obst,Ad,Bd,xd_lb,xd_ub,vd_lb,vd_ub,ud_lb,ud_ub,spline_resolution);

% Develop the Objective Functions
ObjFunc(W,x,u,p,dv);

% Convexity Constraint:
ConvexityConstraint(spline_resolution);
end
