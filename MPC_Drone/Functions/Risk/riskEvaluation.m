function [rfun_x, rfun_y, failure_flag] = riskEvaluation(agent, adversary, radius)
% Parameters:
scale = 1;

% Agent Position:
x = agent(:, 1);
y = agent(:, 2);

% Adversary Position:
x_ = adversary(:, 1);
y_ = adversary(:, 2);

% Distance to Adversary:
d = sqrt((x - x_).^2 + (y - y_).^2);
d_ = d - radius;

% Failure Probability Simulation:
rfun = 1 - exp(scale .* d_);
rfun(rfun < 0) = 0;
simulated_failure = zeros(size(rfun));
for i = 1:numel(rfun)
   simulated_failure(i) = randsrc(1, 1, [0, 1; 1-rfun(i), rfun(i)]);
end

% Evaluate Simulation:
idx = find(simulated_failure == 1, 1);
if isempty(idx)
    rfun_x = d;
    rfun_y = simulated_failure;
    failure_flag = 0;
else
    rfun_x = d(1:idx);
    rfun_y = simulated_failure(1:idx);
    failure_flag = 1;
end

end