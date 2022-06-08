function [] = ConvexityConstraint(spline_resolution)
% Symbolic Variables:
y = sym('y', [spline_resolution+1, 1]);
x = sym('x', [spline_resolution+1, 1]);
m = sym('m', [spline_resolution, 1]);
convexity_constraint = sym(zeros(spline_resolution-1, 1));

% Convexity Constraint:
m(:, 1) = (y(2:end) - y(1:end-1)) ./ (x(2:end) - x(1:end-1));
convexity_constraint(:, 1) = m(2:end) - m(1:end-1);

% Ax <= b:
[A, b] = equationsToMatrix(convexity_constraint, y);

% Generate Function:
disp('Generating Convexity Constraint...')
matlabFunction(A,b,'File',[pwd '/Functions/AutoGenerated/get_conv_constraint'],'Vars',{x},'Outputs',{'A_conv','b_conv'});
end