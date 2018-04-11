function m = MTMModel( )

% q1 = x(1);
% q2 = x(2);
% q3 = x(3);
% q4 = x(4);
% q5 = x(5);
% q6 = x(6);
% q7 = x(7);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% In type, "1" denotes revolute
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% joint; "0" denotes prismatic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% joint
m.DH=[
    % type   alpha   a      d     theta
    % ====   =====   =      =     =====
       1       0     0      0      pi/2;
       1     -pi/2   0      0     -pi/2;
       1       0    -0.2794 0      pi/2;
       1      pi/2  -0.3645 0.1506    0;
       1     -pi/2   0      0         0;
       1      pi/2   0      0      pi/2;
       1      pi/2   0      0      pi/2;
    ];

end

