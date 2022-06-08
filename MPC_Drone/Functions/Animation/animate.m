function [plot, frame, frame_iter] = animate(fig_handle, ax_handle, plot, data, frame, frame_iter)
len = numel(data{1}(:, 1));

%% Static Plots:
% Drone Trajectory:
plot{1}{2}.XData = data{2}(1, :);
plot{1}{2}.YData = data{2}(2, :);

% Obstacle Trajectory:
plot{1}{4}.XData = data{4}(1, :);
plot{1}{4}.YData = data{4}(2, :);

% Sample Points:
plot{2}{1}.XData = data{6}(:, 1);
plot{2}{1}.YData = data{6}(:, 2);

% Spline Points:
plot{2}{2}.XData = data{7}(:, 1);
plot{2}{2}.YData = data{7}(:, 2);

% Spline:
plot{2}{3}.XData = data{7}(:, 1);
plot{2}{3}.YData = data{7}(:, 2);

% Logsurvival:
plot{3}{1}.XData = data{8}(:, 1);
plot{3}{1}.YData = data{8}(:, 2);

% Logsurvival Points:
plot{3}{2}.XData = data{8}(:, 1);
plot{3}{2}.YData = data{8}(:, 2);

% Logsurvival Risk Evaluation:
plot{3}{3}.XData = data{12}(1);
plot{3}{3}.YData = data{9}(1);
plot{3}{4}.XData = data{13}(1);
plot{3}{4}.YData = data{9}(1);

% Constraint Line:
plot{1}{5}.Function = data{5}{1};

% Control Aspect Ratio:
pbaspect(ax_handle(2), [2 1 1]);
pbaspect(ax_handle(3), [2 1 1]);

%% Animate:
frame_limit = 1;
for i = 1:len
    % Drone:
    plot{1}{1}.XData = data{1}(i, 1);
    plot{1}{1}.YData = data{1}(i, 2);

    % Obstacle:
    plot{1}{3}.XData = data{3}(i, 1);
    plot{1}{3}.YData = data{3}(i, 2);
    translate = makehgtform('translate', [data{3}(i, 1), data{3}(i, 2), 0]);
    set(plot{1}{7}, 'Matrix', translate);
    
    % Constraint Lines: (Only Control Over First Node)
%     plot{1}{5}.Function = data{5}{i};
    
    % Risk Projection:
    plot{1}{6}.XData = data{1}(i, 1);
    plot{1}{6}.YData = data{1}(i, 2);
%     plot{1}{6}.UData = data{10}(1, i);
%     plot{1}{6}.VData = data{10}(2, i);
%     plot{1}{6}.AutoScaleFactor = abs(data{9}(i));
    % (Only Control Over First Node)
    plot{1}{6}.UData = data{10}(1, 1);
    plot{1}{6}.VData = data{10}(2, 1);
    plot{1}{6}.AutoScaleFactor = abs(data{9}(1));
    
    % Control Aspect Ratio:
    pbaspect(ax_handle(1), [1 1 1]);
    xlim(ax_handle(1), [-5 5]);
    ylim(ax_handle(1), [-5 5]);
    
    drawnow();
    
    % Frame Limiter:
    if mod(frame_limit, 4) == 0
        % Reset Frame Limiter:
        frame_limit = 0;
        % Save Frame:
        frame(frame_iter) = getframe(gcf);
        frame_iter = frame_iter + 1;
    end
    frame_limit = frame_limit + 1; 
end

end