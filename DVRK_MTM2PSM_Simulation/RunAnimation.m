function RunAnimation( mtm_tip_pos, mtm_tip_ori, psm_tip_pos, psm_tip_ori )

% X = [0;0;0];
% Y = [0;0;0];
% Z = [0;0;0];
% U = [1;0;0];
% V = [0;1;0];
% W = [0;0;1];
% quiver3(X,Y,Z,U,V,W)

    %%%% Showing the two link planar
    %
    % For easier to view the initial pose, it will pause a couple of secs.
    %
%     clf;
%     hold on;
%     title(sprintf('t = %6.4f',0));
%     if ~isempty(x_dsr)
%         plot(x_dsr(1,:), x_dsr(2,:), 'g')
%     end
%     drawRRPlanar(0, x(:,1), model);
%     hold off;
%     pause(2);
% 
%     % Animation
%     %
%     sim_time = 0;
%     tic; % Start stopwatch timer for simulation

%     while sim_time < tspan(end)

        % Interpolate to get the new point at current time:
%         sim_time = toc;
%         xq_ode = interp1(t', x', sim_time, 'spline')';

%         clf;
%         hold on;
%         title(sprintf('t = %6.4f',sim_time));
%         %
%         if ~isempty(x_dsr)
%             plot(x_dsr(1,:), x_dsr(2,:), 'g')
%         end
%         %

subplot(1,2,1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  MTM base frame
arrow3([0 0 0], [1 0 0],'r2o',4,10);
% text(1, 0, 0,'X');
hold on
baseplaneDraw
arrow3([0 0 0], [0 1 0],'g2o',7,14);
% text(1, 0, 0,'Y');
hold on
arrow3([0 0 0], [0 0 1],'b2o',7,14);
% text(1, 0, 0,'Z');
hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MTM tip fram
DrawFrame ( mtm_tip_pos, mtm_tip_ori )
title('MTM');
grid on;
hold off;

axis([-1.5 1.5 -1.5 1.5 -1.5 1.5])
xlabel('x');
ylabel('y');
zlabel('z');
     
drawnow;

subplot(1,2,2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  PSM base frame
R = [-1 0 0;
     0 -1 0;
     0 0 1;
     ];
% R = [0    -1     0;
%      1     0     0;
%      0     0     1]; 
% R = eye(3);
arrow3([0 0 0], R(:,1)','r2o',4,10);
% text(-1,0,0,'X');
hold on
baseplaneDraw
arrow3([0 0 0], R(:,2)','g2o',7,14);
% text(0,-1,0,'X');
hold on
arrow3([0 0 0], R(:,3)','b2o',7,14);
% text(0,0,1,'X');
hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PSM tip fram
DrawFrame ( psm_tip_pos, psm_tip_ori )
title('PSM');
grid on;
hold off;

axis([-1.5 1.5 -1.5 1.5 -1.5 1.5])
xlabel('x');
ylabel('y');
zlabel('z');
     
drawnow;
%         pause(0.5);

        % Update simulation time
%         sim_time = toc;
%     end

end