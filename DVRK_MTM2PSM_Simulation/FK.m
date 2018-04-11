function [ Tip, J, psm_fixed_tip ] = FK( m, q )
% INPUT:
% OUTPUT:

% Description:

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T             = []; %MTM:4*4*7
[Joint_num,~] = size(m.DH);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Improve DH table
for i = 1:Joint_num
    if m.DH(i,1) == 1
        m.DH(i,5) = m.DH(i,5) + q(i);
    end
    if m.DH(i,1) == 2
        m.DH(i,4) = m.DH(i,4) + q(i);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Calculate transformation matrices

for i = 1:Joint_num % 
    T(:,:,i) = [
               cos(m.DH(i,5))                  -sin(m.DH(i,5))                 0                  m.DH(i,3)                 ;
               sin(m.DH(i,5))*cos(m.DH(i,2))   cos(m.DH(i,5))*cos(m.DH(i,2))   -sin(m.DH(i,2))    -sin(m.DH(i,2))*m.DH(i,4) ;
               sin(m.DH(i,5))*sin(m.DH(i,2))   cos(m.DH(i,5))*sin(m.DH(i,2))   cos(m.DH(i,2))     cos(m.DH(i,2))*m.DH(i,4)  ;
               0                               0                               0                  1                         ;
               ];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Cartesian position
HT_tip2base = HomoTran (T, Joint_num, 0);
% 
% HT_tip2base = HT_tip2base*[0 -1 0 0; 
%                            1  0 0 0;
%                            0  0 1 0;
%                            0 0 0 1];

Tip.pos     = HT_tip2base(1:3,4);
Tip.ori     = HT_tip2base(1:3,1:3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Jacobian matrix consists of
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% translation component and
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% orientation component.

[J,psm_fixed_tip] = ComputeJacobian(T, m, HT_tip2base);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Calculate the homogeneous transformation
% Example: HomoTran (T, 6, 0) = the homogeneous
% transformation from Frame 6 to Frame 0. Frame 0 means the base frame.

function  HT = HomoTran (T, StartFrame, TargetFrame)
     HT = eye(4,4);
     for i = (TargetFrame+1):StartFrame
         HT = HT*reshape(T(:,:,i),4,4);
     end
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Calculate Jacobian
% function J = ComputeJacobian(T, m, Tip)
%     Z0 = [0;0;1];
%     P0 = [0;0;0];
%     if m.DH(1,1) == 1
%         J_first_column = [cross(Z0,(Tip.pos-P0));Z0];
%     end
%     if m.DH(1,1) == 2
%         J_first_column = [Z0;zeros(3,1)];
%     end
%     
%     J             = [];
%     [Joint_num,~] = size(m.DH);
%     for i=2:Joint_num
%         
%         HT_temp    = HomoTran (T, i-1, 0);
%         Pos_vector = HT_temp(1:3,4);
%         Z_vector   = HT_temp(1:3,3);
%         
%         if m.DH(i,1) == 1
%             J_temp_column = [cross(Z_vector,(Tip.pos-Pos_vector));Z_vector];
%             J             = [J J_temp_column];
%         end
%         if m.DH(i,1) == 2
%             J_temp_column = [Z_vector; zeros(3,1)];
%             J             = [J J_temp_column];
%         end
%     end
%     J = [J_first_column J];
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Calculate Jacobian
function [J,psm_fixed_tip]  = ComputeJacobian(T, m, HT_tip2base )
    Additional_DH_row = [-pi/2; 0; 0.0102; pi/2]; %alpha;a;d;theta
    HT_real_tip       = HT_tip2base * [
               cos(Additional_DH_row(4))                             -sin(Additional_DH_row(4))                            0                             Additional_DH_row(2)                            ;
               sin(Additional_DH_row(4))*cos(Additional_DH_row(1))   cos(Additional_DH_row(4))*cos(Additional_DH_row(1))   -sin(Additional_DH_row(1))    -sin(Additional_DH_row(1))*Additional_DH_row(3) ;
               sin(Additional_DH_row(4))*sin(Additional_DH_row(1))   cos(Additional_DH_row(4))*sin(Additional_DH_row(1))   cos(Additional_DH_row(1))     cos(Additional_DH_row(1))*Additional_DH_row(3)  ;
               0                               0                               0                  1                         ;
                                      ];
    HT_real_tip_pos = HT_real_tip(1:3,4);
    
    psm_fixed_tip.pos = HT_real_tip(1:3,4);
    psm_fixed_tip.ori = HT_real_tip(1:3,1:3);
    
    J             = [];
    [Joint_num,~] = size(m.DH);
    for i=1:Joint_num
        
        HT_temp    = HomoTran (T, i, 0);
        Pos_vector = HT_temp(1:3,4);
        Z_vector   = HT_temp(1:3,3);
        
        if m.DH(i,1) == 1
            J_temp_column = [cross(Z_vector,(HT_real_tip_pos-Pos_vector));Z_vector];
            J             = [J J_temp_column];
        end
        if m.DH(i,1) == 2
            J_temp_column = [Z_vector; zeros(3,1)];
            J             = [J J_temp_column];
        end
    end
end