function [delta, n, q_f, qo_f] = RiskSourceFormulation(drone_states_ic, drone_states_future, drone_states_future_old, obstacle_states_ic, Th, Nodes)
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
        n = qo_f - q_f;
        scale = sqrt(n(1, :).^2 + n(2, :).^2);
        
        % Drone Relative to Obstacle:
        q_ = qo_f - q;
        
        % Risk Source: Linearized Distance
        delta(i,:) = (dot(q_, n) ./ dot(n, n)) .* scale;
    end  
end

