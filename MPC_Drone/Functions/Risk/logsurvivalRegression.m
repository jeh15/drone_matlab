function [x_, y_] = logsurvivalRegression(x, y, w)
% Setup:
dim = numel(x);
x_ = x;
yd = log(1-y);
% Hessian:
H = 2 .* w .* eye(dim);
% Gradient:
f = -2 .* w .* ones(dim, 1) .* yd;
% Constraints:
[A, b] = get_conv_constraint(x.');
% Bounds:
lb = -inf*ones(dim, 1);
ub = zeros(dim, 1);
% Optimize:
y_ = quadprog(H, f, A, b, [], [], lb, ub);
end