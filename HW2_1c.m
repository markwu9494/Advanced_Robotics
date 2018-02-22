%Homework2-1-c-simulation of manipulability ellipsoid
clc
clear all

%% initialization
a = [ -80 -60 -40 -20 -10];
b = a * (-2); %form a isosceles triangle

%% figure plotting
for i = 1:1:5
    q2 = a(i);
    q3 = b(i);
    s2 = sind(q2);
    s3 = sind(q3);
    c2 = cosd(q2);
    c3 = cosd(q3);
    d = sqrt(2)/2;
    J = [ -d*s2-d*s2*c3-d*c2*s3 -d*c2*s3-d*s2*c3;...
          s2*s3-c2*c3-c2 -c2*c3+s2*s3 ];
    x3 = d*c2+d*c2*c3-d*s2*s3;
    z3 = 1-c2*s3-c3*s2-s2;
    x2 = d*c2;
    z2 = 1-s2;
    x1 = 0;
    z1 = 1;
    figure(1)
    plot([0 x1 x2 x3],[0 z1 z2 z3],'color','k');
    hold on
    plot([x1 x2 x3],[z1 z2 z3],'o','color','r');
    C1 = J*J';
    D1 = inv(C1);
    syms('x','z');
    p = [ x-x3;...
          z-z3 ];
    E_v = p'*D1*p;
    E_f = p'*C1*p;
    ezplot(E_v==1,[-0.5 1.6 -1.5 3.5]);
    title('Velocity manipulability ellipsoid');
    set(gcf,'color','white')
    figure(2)
    plot([0 x1 x2 x3],[0 z1 z2 z3],'color','k');
    hold on
    plot([x1 x2 x3],[z1 z2 z3],'o','color','r');
    ezplot(E_f==1,[-8 11 -2 4]);
    title('Force manipulability ellipsoid');
    set(gcf,'color','white')
end
    