function [xPoints, yPoints] = logsurvivalRegression(xValues, yValues, weightVect)
iterLimit = 10000;
nNodes = numel(yValues);
nlp = DirectCollocation(nNodes);
%% Variables:
yDesired = log(1-yValues);
% Initializing y Variables:
for i = 1:numel(xValues)
    y(i) = nlp.addVariable(0,-Inf,0,'Description','Variable: y', 'Length', 1);
end
%% Linear Fit Math:
% Counting Variables:
iter = 1;
% Pre-allocation:
yp = PlusBinaryOperatorNode.empty(nNodes, 0);
yError = PlusBinaryOperatorNode.empty(nNodes, 0);
% Linear Fit
for i = 1:nNodes
    if xValues(i) == xValues(iter)
        yp(i) = y(iter);
        yError(i) = yDesired(i) - yp(i);
        iter = iter + 1;
    elseif xValues(i) < xValues(iter)
        a = (xValues(i)-xValues(iter-1))./(xValues(iter)-xValues(iter-1));
        yp(i) = y(iter-1) + (y(iter)-y(iter-1)).*a;
        yError(i) = yDesired(i) - yp(i);
    else
        iter = iter + 1;
        a = (xValues(i)-xValues(iter-1))./(xValues(iter)-xValues(iter-1));
        yp(i) = y(iter-1) + (y(iter)-y(iter-1)).*a;
        yError(i) = yDesired(i) - yp(i);
    end
end
% % Over-estimation Constraint:
% for i = 1:numel(yError)
%     nlp.addConstraint(0,yError(i),Inf);
% end
% Slope:
for i = 1:numel(y)-1
    m(i) = (y(i+1)-y(i))./(xValues(i+1)-xValues(i));
end
% Slope Constraint: Convex
for i = 1:numel(m)-1
    nlp.addConstraint(-Inf,m(i+1)-m(i),0);
end
% Weights:
total_points = sum(weightVect);
weight = (weightVect(:))./total_points;
%% Sum Error:
error = 0;
for i = 1:numel(yError)
    if i == 1
        error = error + weight(1).*(yError(i)).^2;
    elseif i == numel(yError)
        error = error + weight(end).*(yError(i)).^2;
    else
        error = error + (weight(i-1)+weight(i)).*((yError(i)).^2);
    end
end
%% Objective:
% Minimize Error
nlp.addObjective(error, ...
    'Description','Minimize Error Objective')
optim = Ipopt(nlp);
optim.options.ipopt.max_iter = iterLimit;
optim.export;
optim.solve;
%% Post Processing:
% Solution:
yPoints = squeeze(eval(y));
xPoints = xValues;
end