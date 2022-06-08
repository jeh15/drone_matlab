function hooks = splineFit(points, numSplinesPerDim, weights, convex, bound)
    %"points" is an array of the points that are to be fitted:
    %[x1(1), x2(1), ... xn(1), y(1);
    % x1(2), x2(2), ... xn(2), y(2);
    % .............................
    % x1(N), x2(N), ... xn(N), y(N)]
    %Each point has n x-values corresponding to n dimensions or n sources
    %of risk. Each point also has 1 y-value corresponding to its failure
    %state or logS value.
    
    %The function returns an array of (numSplinesPerDim + 1)^n "hooks" that
    %are the border points of the spline. The hooks array has the same
    %format as the input points array. These hooks can then be used with
    %the "interp" function to predict the y-value of any other
    %n-dimensional point.
    
    %The convex and bound flags determine whether the spline fit will be
    %constrained to be convex (slopes are monotonically increasing or
    %decreasing), and bounded (all hooks have y-values between 0 and 1)
    
    %% Set up:
    sz = size(points);
    numPoints = sz(1);
    numDims = sz(2) - 1;
    outputs = points(:, numDims + 1);
    points = points(:, 1:numDims);
    vars = points';
    
    mins = inf * ones(1, numDims);
    maxs = -inf * ones(1, numDims);
    for i = 1 : numPoints
        for j = 1 : numDims
            if points(i, j) < mins(j)
                mins(j) = points(i, j);
            end
            if points(i, j) > maxs(j)
                maxs(j) = points(i, j);
            end
        end
    end
    maxs = maxs + 0.1;
    mins = mins - 0.1;
    
    for i = 1 : numPoints
        for j = 1 : numDims
            points(i, j) = points(i, j) - mins(j);
            if maxs(j) ~= mins(j)
                points(i, j) = points(i, j)/(maxs(j) - mins(j));
            end
        end
    end

    % Number of spline points:
    numHooksPerDim = 1 + numSplinesPerDim;
    hookGap = 1 / (numHooksPerDim - 1);
    
    h = numHooksPerDim;
    n = numDims;

    xPoints = zeros(h^n, n);
    clock = zeros(1, n);
    for i = 1 : h^n
        %clock
        xPoints(i, :) = (hookGap * clock) .* (maxs - mins) + mins;
        ind = 1;
        clock(ind) = clock(ind) + 1;
        while clock(ind) == h && i < h^n
            clock(ind) = 0;
            clock(ind + 1) = clock(ind + 1) + 1;
            ind = ind + 1;
        end
    end    
    
    %Generate Weights:
    if weights == 1
        weights = multidimBin(points, xPoints);
        weights = diag(weights);
    else
        weights = eye(h^n, h^n);
    end
    
    H = zeros(h^n, h^n);
    f = zeros(h^n, 1);
    
    for p = 1 : numPoints
        hookList = zeros(1, numDims);
        cList = zeros(1, numDims);
        dList = zeros(1, numDims);
        for j = 1 : numDims
            x = floor(points(p, j) / hookGap);
            if x == h - 1 %takes care of the case when a point is at the max
                x = x - 1;
            end
            hookList(1, j) = x;
            dList(1, j) = (points(p, j) - hookGap * hookList(1, j))/hookGap;
        end
        %hookList
        base = compress(hookList, h);
        locList = zeros(1, 2^numDims);
        locList(1) = base;
        
        prodList = zeros(1, 2^numDims);
        
        binList = zeros(1, numDims);
        prod = 1;
        for k = 1 : numDims
            prod = prod * (1 - dList(k));
        end
        prodList(1) = prod;
        for j = 1 : 2^numDims - 1
            binList(1) = binList(1) + 1;
            for k = 1 : numDims
                if binList(k) == 2
                    binList(k) = 0;
                    binList(k + 1) = binList(k + 1) + 1;
                end
            end
            prod = 1;
            for k = 1 : numDims
                if binList(k) == 1
                    prod = prod * dList(k);
                else
                    prod = prod * (1 - dList(k));
                end
            end
            prodList(j + 1) = prod; 
            nList = hookList + binList;
            locList(j + 1) = compress(nList, h);
        end

        for i = 1 : size(locList, 2)
            for j = i : size(locList, 2)
                hook1 = locList(1, i);
                prod1 = prodList(1, i);
                hook2 = locList(1, j);
                prod2 = prodList(1, j);
                H(hook1, hook2) = H(hook1, hook2) + prod1 * prod2;
            end
        end
        
        %F TIME
        for i = 1 : size(locList, 2)
            hook1 = locList(1, i);
            prod1 = prodList(1, i);
            f(hook1, 1) = f(hook1, 1) + (-2 * outputs(p) * prod1);
        end
    end
    
    H = (H + H') / 2;
    
    H = 2 * H;
    
    H = weights * H;
    
    % Added an extra bound [-Inf 0]
    boundA = eye(h^n, h^n);
    if bound == 1
        boundL = 0.001 * ones(h^n, 1);
        boundU = 0.999 * ones(h^n, 1);
    elseif bound == 2
        boundL = -Inf * ones(h^n, 1);
        boundU = zeros(h^n, 1);
        bound = 1;
    end
    
    planeA = zeros((n - 1)^2 * (h - 1)^n, h^n);
    root = zeros(1, n);
    root(1) = -1;
    v = 1;
    for i = 1 : h^n
        ind = 1;
        root(ind) = root(ind) + 1;
        while root(ind) == h
            root(ind) = 0;
            root(ind + 1) = root(ind + 1) + 1;
            ind = ind + 1;
        end
        %Checks that simplex is not on the border!
        if max(root) < h - 1
            %Iterates through every possible connection from the root
            for dim = 1 : n - 1
                %Checks all corresponding connections to that connection
                for adj = 1 : n
                    if dim ~= adj
                        a = h^(dim - 1);
                        b = h^(adj - 1);
                        planeA(v, i + a + b) = 1;
                        planeA(v, i + b) = -1;
                        planeA(v, i + a) = -1;
                        planeA(v, i) = 1;
                        v = v + 1;
                    end
                end
            end
            %Moves to the next row of A only when one row has completed
            %(corresponding to one non-border simplex)
        end         
            
    end    
    planeL = zeros((n - 1)^2 * (h - 1)^n, 1);
    planeU = zeros((n - 1)^2 * (h - 1)^n, 1);
    
    convA = zeros(n * (h - 2), h^n);
    v = 1;
    for dim = 1 : n
        a = h^(dim - 1);
        for i = 1 : h - 2
            convA(v, a * (i + 2)) = 1;
            convA(v, a * (i + 1)) = -2;
            convA(v, a * (i)) = 1;
            v = v + 1;
        end
    end
    convL = -inf * ones(n * (h - 2), 1);
    convU = zeros(n * (h - 2), 1);
    A = [];
    l = [];
    u = [];

    if numDims > 1
        A = [planeA];
        l = [planeL];
        u = [planeU];
    end
    if convex
        A = [A; convA];
        l = [l; convL];
        u = [u; convU];
    end
    if bound
        A = [A; boundA];
        l = [l; boundL];
        u = [u; boundU];
    end
    
    %% Run the Quadratic Program:
    %Quadprog:
    A = [A; -A];
    b = [u; -l];
    
    x = quadprog(H, f, A, b, [], [], [], []);
    
%     keyboard;
    
    yPoints = x;
    
    hooks = [xPoints, yPoints];
end