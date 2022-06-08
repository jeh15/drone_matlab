function [] = Animation(j, s, Dims, Num_Obs, h1, Ts, r_min, r_min2, qo_i, q_i, q, q_des, filename)
    cla
    if Dims == 2
        axis equal
        hold on
        axis off
        set(gcf,'color','white')
        xlim([-2.5,2.5]);
        ylim([-2.5,2.5]);
        

        plot(q_des(1), q_des(2),'-p','MarkerFaceColor','white','MarkerEdgeColor','black','MarkerSize',15)
        rectangle('Position',[q_i(1)-r_min, q_i(2)-r_min, r_min*2, r_min*2],'Curvature',1,'FaceColor','b','EdgeColor','b')
        plot(q(1,:),q(2,:),'k--')
        % Attempt to Draw Half-Space:
        d_ = q_i(:, :) - qo_i(:, :);
        plot(d_(1, :), d_(2, :));
        for i = 1:Num_Obs
            rectangle('Position',[qo_i(1,i)-r_min, qo_i(2,i)-r_min, r_min*2, r_min*2],'Curvature',1,'FaceColor','r','EdgeColor','r')
            rectangle('Position',[qo_i(1,i)-r_min2, qo_i(2,i)-r_min2, r_min2*2, r_min2*2],'Curvature',1,'EdgeColor','k')
        end
    else
        for i = 1:Num_Obs
            plot(q_des(1,i), q_des(2,i), q_des(3,i),'-p','MarkerFaceColor','white','MarkerEdgeColor','black','MarkerSize',15)
            plot3(q_i(1,i),q_i(2,i),q_i(3,i),'.b','MarkerSize',40)
            plot3(q(1,:,i),q(2,:,i),q(3,:,i),'k--')
            sphere2 = surfl(s.x*r_min2 + q_i(1,i), s.y*r_min2 + q_i(2,i), s.z*r_min2 + q_i(3,i));
            set(sphere2, 'FaceAlpha', 0.1, 'FaceColor', [0 1 0])
        end
    end
    drawnow;
    
   % GIF:
   frame = getframe(h1);
   im = frame2im(frame);
   [imind,cm] = rgb2ind(im,256);
   if j == 1
       imwrite(imind,cm,filename,'gif','DelayTime',Ts,'Loopcount',inf);
   elseif j == numel(j)
       imwrite(imind,cm,filename,'gif','DelayTime',Ts,'WriteMode','append');
   else
       imwrite(imind,cm,filename,'gif','DelayTime',Ts,'WriteMode','append');
   end  
end