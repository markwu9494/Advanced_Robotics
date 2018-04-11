function DrawFrame ( origin_pos, frame_ori )

X = origin_pos;
Y = origin_pos;
Z = origin_pos;

arrow3(X', frame_ori(:,1)','r2o',7,14);
arrow3(Y', frame_ori(:,2)','g2o',7,14);
arrow3(Z', frame_ori(:,3)','b2o',7,14);

text(X(1),X(2),X(3),'x');
% scale = 1;
% 
% U =   scale * frame_ori(:,1);
% V =   scale * frame_ori(:,2);
% W =   scale * frame_ori(:,3);

% h = quiver3(X,Y,Z,U,V,W);
% 
% h.Color = 'red';
% h.LineWidth = 1.5;
% h.MaxHeadSize = 0.5;
% h.AutoScale = 'off';
% h.DisplayName = 'x';
end

