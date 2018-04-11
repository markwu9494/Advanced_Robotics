function m = PSMModel( )

% q1 = x(1);
% q2 = x(2);
% d3 = x(3);
% q4 = x(4);
% q5 = x(5);
% q6 = x(6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% In type, "1" denotes revolute
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% joint; "0" denotes prismatic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% joint
m.DH=[
    % type   alpha   a      d         theta
    % ====   =====   =      =         =====
       1      pi/2   0      0         pi/2;
       1     -pi/2   0      0        -pi/2;
       2      pi/2   0     -0.4318       0;
       1         0   0      0.4162       0;
       1     -pi/2   0      0        -pi/2;
       1     -pi/2   0.0091 0        -pi/2;
    ];

end

