function [qd, ud, fval, design_vector] = RunMPCRisk(Th,Nodes,qd_i,qo_i,qd_des,qd_prev,xd_lb,xd_ub,m,b,x0)
   % Optimization Options
    options = optimset('Display','off');  
    
    % Develop the Constraints Matrix and Vector
    [Aeq,beq] = EqualityConstraints(qd_i);
    [Aiq,biq] = InequalityConstraints(qd_i, qd_prev, qo_i, m, b);
    
    % Develop The Objective Function Matrix and Vector
    [H, f] = ObjectiveFunction(qd_des);
    
    % DEBUG:
    x_lim = 5;
    dx_lim = 1.75;
    u_lim = 5;
    states_lb = [-x_lim; -x_lim; -dx_lim; -dx_lim];
    states_ub = [x_lim; x_lim; dx_lim; dx_lim];
    control_lb = [-u_lim; -u_lim];
    control_ub = [u_lim; u_lim];
    risk_lb = -Inf .* ones(Nodes, 1);
    risk_ub = zeros(Nodes, 1);
    lb = [repmat(states_lb, Nodes, 1); repmat(control_lb, Nodes, 1); risk_lb];
    ub = [repmat(states_ub, Nodes, 1); repmat(control_ub, Nodes, 1); risk_ub];
    [design_vector, fval, exitflag, output] = quadprog(H,f,Aiq,biq,Aeq,beq,lb,ub,[],options);
    if exitflag <= 0
        keyboard;
    end
    
    % Trajectory Bus:
    dt = Th/(Nodes - 1);
    trajectory = zeros(Nodes, 33);  
    
    % Seperate the states in the design vector
    M = size(qd_i,1);           
    O = size(qd_i,1)/2;         % Size of bounding vector
    qd = zeros(M,Nodes);        % Initialize the desired states
    ud = zeros(O,Nodes);        % Initialize the desired control inputs
    for i = 1:M
        if ~isempty(design_vector)
          qd(i,:) = design_vector(i:M:M*Nodes).';
        else   
%             dist = qd_i - qo_i;
%             r_dist = sqrt(dist(1)^2 + dist(2)^2);
%             if qd_i(1) <= xd_lb(1) || qd_i(1) >= xd_ub(1) || qd_i(2) <= xd_lb(2) || qd_i(2) >= xd_ub(2)
%                 warning('Cartesian Bounds Violated, Optimization did not produce results');
%                 keyboard
%             elseif r_dist < r1
%                 warning('Minimum distance from obstacle violated, optimization did not produce results');
%                 keyboard
%             else
%                 warning('Optimization can not find a real solution');
%                 keyboard
%             end
            qd(1:M/2,:) = qd_i(1:M/2,:).*ones(M/2,Nodes);
        end
    end
    
    % Seperate the forces in the design vector
    for j = 1:O
        try
            ud(j,:) = design_vector(M*Nodes+j:O:O*Nodes+M*Nodes).';
        catch
        end
    end
    
    % Replace variables with the known results [dt, x, dx, y, dy, z, dz] = [1, 2, 3, 10, 11, 18, 19]
    trajectory(:,1) = dt;
    trajectory(:,2:3) = [qd(1,:); qd(3,:)].';
    trajectory(:,10:11) = [qd(2,:); qd(4,:)].';
   
    % Find further numeric derivatives
    for i = 1:6
        trajectory(:,i+3) = gradient(trajectory(:,i+2));
        trajectory(:,i+11) = gradient(trajectory(:,i+10));
    end
    trajectory = trajectory(1:end,:);
    % Create a csv file for python
    writematrix(trajectory,'TEST.csv')
    
end
