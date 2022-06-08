function [xPoints, yPoints] = riskcurveRegression(delta, dataPoints, spline_resolution)
    %% Create a Linear Fit Spline:
    % A function that fits a n-segmented linear spline to some set of data.
    
    % This function takes in (xVals) which are the x-values of the data
    % set,(yVals) which are the y-values of the data set, and (numSplines)
    % which is an integer representing the number of splines that the data
    % will be matched to.
    
    % This function returns (xPoints) which are the x-values of the points
    % that join the splines, and (yPoints) which are the y-values of the
    % points that join the splines.
    
    %% Set up:
    yVals = dataPoints;
    xVals = delta;
    
    % Number of spline points:
    numPoints = 1 + spline_resolution;
    
    %Number of data points:
    numVals = length(xVals);
    
    %Sort the x-values of the data in increasing order, then the y-values
    %in the corresponding order:
    [xVals, I] = sort(xVals);
    newY = zeros(1, numVals);
    for i = 1 : numVals
        newY(i) = yVals(I(i));
    end
    yVals = newY;
    
    %% Variables:
    
    % x-values of spline points, evenly spaced along interval:
    xPoints = linspace(xVals(1), xVals(numVals), numPoints);
    
    % Initialize the H and f matrices as all zeros:
    H = [zeros(numPoints, numPoints)];
    f = [zeros(numPoints, 1)];
    
    %% Linear Fit Math:
    
    % Counting variable (specifies which spline a certain data point
    % should belong to):
    loc = 1;
    
    % Explanation of variables:
    % a is an intermediate expression used to clean up some of the math;
    % yd is a data point we are trying to fit to;
    % y1Sq is the expression multiplied by (yPoints(loc - 1))^2 in the H matrix;
    % y1y2 is the expression multiplied by (yPoints(loc - 1)) *
    % (yPoints(loc)) in the H matrix;
    % y2Sq is the expression multiplied by (yPoints(loc))^2 in the H
    % matrix;
    % y1 is the expression multiplied by yPoints(loc - 1) in the f matrix;
    % y2 is the expression multiplied by yPoints(loc) in the f matrix;
    
    % Iterate across the data points:
    for i = 1 : numVals
        % This while loop checks if the current data point is outside the
        % range of the spline it is being fitted for, and if so, moves to
        % the next-highest spline:
        while loc < numPoints && xVals(i) >= xPoints(loc)
            loc = loc + 1;
        end
        a = (xVals(i) - xPoints(loc - 1))/(xPoints(loc) - xPoints(loc-1));
        yd = yVals(i);
        y1Sq = 1 + a^2 - 2*a;
        y1y2 = a - (a^2);
        y2Sq = a^2;
        y1 = -2 * yd + 2 * a * yd;
        y2 = -2 * a * yd;
        
        % Add the computed values to the H and f matrices in the
        % appropriate positions (y1y2 is placed into 2 different positions
        % so that the Hessian remains symmetric):
        H(loc - 1, loc-1) = H(loc-1, loc-1) + y1Sq;
        H(loc, loc-1) = H(loc, loc-1) + y1y2;
        H(loc-1, loc) = H(loc-1, loc) + y1y2;
        H(loc, loc) = H(loc, loc) + y2Sq;
        f(loc-1, 1) = f(loc-1, 1) + y1;
        f(loc, 1) = f(loc, 1) + y2;
    end
    
    %Double the H matrix to account for the 1/2 factor
    H = 2 * H;
    
    %Constraints
    A = eye(numPoints);
    b = zeros(numPoints, 1);
    l = 0.00.*ones(numPoints, 1);
    u = 1.00.*ones(numPoints, 1);
    
    %% Run the Quadratic Program:
    % Quadprog:
    yPoints = quadprog(H, f, [], [], [], [], l, u);
    
end