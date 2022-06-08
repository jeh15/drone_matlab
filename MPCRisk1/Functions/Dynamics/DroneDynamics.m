function dq = DroneDynamics(t,q,t_mpc,u_mpc,A,B)
    % Interpolate the control inputs
    u = interp1(t_mpc,u_mpc.',t).'; 
    
    % Setup for ODE45
    dq = A*q + B*u;
end