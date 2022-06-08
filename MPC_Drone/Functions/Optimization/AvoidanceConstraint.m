function [ciq_avoid] = AvoidanceConstraint(drone_states_ic, drone_states_future, drone_states_future_old, obstacle_states_ic, r1, Th, Nodes)
    % Uses the current position of the drone, and the expected position of
    % the obstace to develop linear constraints keeping the drone out of
    % the circle by linearizing minimin radius point along the distance vector    

    % Create Time Vectors
    Tvec = linspace(0,Th,Nodes);
    
    % Grab only the Position Data
    Dims = size(drone_states_ic,1)/2;
    Obst_Dims = size(obstacle_states_ic,2);
    q_ic = drone_states_ic(1:Dims,:);
    q_prev = drone_states_future_old(1:Dims,:);
    qo_ic = obstacle_states_ic(1:Dims,:);
    dqo_ic = obstacle_states_ic(Dims+1:end,:);
    q = drone_states_future(1:Dims,:);
    
    % Find the constraints for each Obstacle
    for i = 1:Obst_Dims
        % Take the previous trajectory
        q_f = [q_ic, q_prev(:,2:end)]; 
        
        % Use Obstacle Velocity to Estimate the Obstacle Positions
        qo_f = qo_ic(:,i) + Tvec.*dqo_ic(:,i);        
        
        % Distance Between Agent and Obstacle
        q_dist = q_f - qo_f;

        % Scale the Vector
        for j = 1:Nodes
            u(:,j) = q_dist(:,j)/norm(q_dist(:,j));
        end
        
        % Hard Constraint Intercept & Equation
        intercept = qo_f + r1.*u;
        ciq_avoid(i,:) = -dot(q_dist,q - intercept);
        
        keyboard;
    end  
end

