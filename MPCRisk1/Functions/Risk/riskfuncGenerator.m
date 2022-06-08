function planeEqnCell = riskfuncGenerator(hooks, numSpline)
%Finds parameters used to construct a plane

%Algorithm Parameters:
resolution = 2;                             %Resolution of approximation
numPlane = numSpline^2 * resolution;        %Number of planes
planeEqnCell = cell(numPlane, 1);           %Pre-allocation
stop = 0;                                   %stop condition
offSet = numSpline + 1;                     %Offset for point calculation

%Iter Variables:
loopIter = 0;
cellIter = 1;

%Plane Parameter Generation:
while stop ~= 1
    for i = 1:numSpline
        num1 = i + offSet*loopIter;
        num2 = num1 + 1;
        num3 = num1 + offSet;
        num4 = num3 + 1;
        for j = 1:resolution
            if j == 1
                A = hooks(num1, :);
                B = hooks(num2, :);
                C = hooks(num3, :);
            else
                A = B;
                B = C;
                C = hooks(num4, :);
            end
            n = cross(B - A, C - B);    %Normal Vector
            n = n./norm(n);             %Unit Vector
            d = dot(-n, A);
            
            %Plane Parameters:
            planeEqnCell{cellIter} = [n(1), n(2), n(3), d];
            
            %Cell Iteration:
            cellIter = cellIter + 1;
        end
    end
    
    loopIter = loopIter + 1;
    
    if cellIter == numPlane + 1
        stop = 1;
    end
    
end
end