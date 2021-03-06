function [] = ObjFunc(W,x,u,s,dv)
%% Objective Function Forumlation
% Create Symbolic vars for desired states
x_d = sym('x_d',[size(x,1),1]);

% Find the error (x_d is a vector, x is a matrix)
x_e = x_d - x;

% Objective Weights:
weight_distance = W(1);
weight_force = W(2);
weight_risk = W(3);

% Objective Function:
J_task = weight_distance .* ((x_e).^2);
J_effort = weight_force .* (u.^2);
J_risk = -weight_risk .* s;
J = sum(sum(J_task.', 2) + sum(J_effort.', 2) + sum(J_risk.', 2), 1);

% Find the Matrix components  
H = double(hessian(J,dv));
f_obj = jacobian(J,dv);
f = subs(f_obj,dv,zeros(size(dv)));

% Create a Function to take in desired states
disp('Generating Objective Function...')
matlabFunction(H,f,'File',[pwd '/Functions/AutoGenerated/ObjectiveFunction'],'Vars',{x_d},'Outputs',{'H','f'});
end