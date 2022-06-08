function [Tsim,qd_sim,qo_sim_total,dt] = Simulate(Th,Ts,Nodes,qd_i,qo_i,ud,uo,Ad,Bd,Ao,Bo)
    % Options
    options = odeset('RelTol',1e-5);

    % Time vector
    Tvec = linspace(0,Th,Nodes);
    Tvec_control = linspace(0, Tvec(2), Nodes);
    dt = Tvec_control(2) - Tvec_control(1);
    
    % Update Initial Conditions at Update Rate time
    [Tsim, qd_sim] = ode45(@(t,q) DroneDynamics(t,q,Tvec,ud,Ad,Bd), Tvec_control, qd_i, options);
    
    % Develop the Obstacle Motion
    Obst_num = size(qo_i,2);
    qo_sim_total = [];
    for i = 1:Obst_num
        uo_current = uo(:,i);
        [~, qo_sim] = ode45(@(t,q) ObstacleDynamics(t,q,uo_current,Ao,Bo), Tvec_control, qo_i(:,i), options);
        qo_sim_total = [qo_sim_total, qo_sim];
    end
end

