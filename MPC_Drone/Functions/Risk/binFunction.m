function [weight_vector] = binFunction(data, bin_points, spline_resolution)
data = sort(data);
j = 1;
assigned_flag = 0;
weight_vector = zeros(spline_resolution+1, 1);
for i = 1:numel(data)
    while assigned_flag == 0
        if data(i) <= bin_points(j+1)
            weight_vector(j) = weight_vector(j) + 1;
            weight_vector(j+1) = weight_vector(j+1) + 1;
            assigned_flag = 1;
        else
            j = j + 1;
        end
    end
    assigned_flag = 0;
end
end