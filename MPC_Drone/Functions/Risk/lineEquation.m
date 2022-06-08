function [slope, y_int] = lineEquation(xValues, yValues)
%Pre-allocation:
size = numel(yValues)-1;
slope = zeros(size, 1);
y_int = slope;
for i = 1:size
    P = polyfit(xValues(i:i+1),yValues(i:i+1),1);
    slope(i) = P(1);
    y_int(i) = P(2);
end
end
