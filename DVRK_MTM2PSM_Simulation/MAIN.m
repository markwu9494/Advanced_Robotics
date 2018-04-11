clear all; 
clc;

%load data
% data_folder = './';
load('dvrk_mtm_psm.mat');


dt = 0.001;
scale = 0.5; %translation scale from mtmFixedTip to psmTip
k1  = 500;
k2  = 500;
K  = diag([k1,k1,k1,k2,k2,k2],0); % Gain matrix

%%

% mtm_tip_pos       = []; % MTM tip postion 3*2400
[~, mtm_data_num] = size(mtm_q);


[mtm_tip_initial,~,~] = FK(MTMModel( ), mtm_q(:,1));  % Compute the tip information at MTM in the first sampling
mtm_tip_k = mtm_tip_initial;                        % mtm_tip_k denotes the "old" MTM tip information. "k" means "old"; "kp1", i.e., k+1, means "new". 

[~, ~, psm_fixed_tip_k] = FK(PSMModel( ),psm_q_initial); % Compute the fixed tip information at PSM in the first sampling
psm_fixed_tip_k_pos = psm_fixed_tip_k.pos;               % PSM fixed tip "old" position
psm_q_k = psm_q_initial;                                 % PSM "old" joint angle space(vector)

error_pos = [];                                          % Position error for verifying inverse kinematics
error_ori = [];                                          % Orientation error for verifying inverse kinematics

for i = 1:mtm_data_num-1
  
[mtm_tip_kp1, ~, ~] = FK(MTMModel( ), mtm_q(:,i+1));     % Compute the tip information at MTM, considered as "new" info
%  mtm_tip_pos = [mtm_tip_pos mtm_tip.pos];              % Combine the
                                                         % position information, used for verifying forward kinematics
% % 
[ x_d, dx_d ] = TeleopRelationship( mtm_tip_k, mtm_tip_kp1, psm_fixed_tip_k_pos, dt ,scale); % Compute inverse kinematics inputes.
mtm_tip_k = mtm_tip_kp1;                                 % Save "old" MTM tip info for next loop
psm_fixed_tip_k_pos = x_d(1:3,4);                        % 

% 
% x_d  = psm_x_dsr(:,:,i);
% dx_d = psm_xdot_dsr(:,i);

[~, psm_J, psm_fixed_tip] = FK(PSMModel( ), psm_q_k); % psm_tip->joint6,psm_fixed_tip->joint7

psm_q_kp1 = IK( x_d, dx_d, psm_J, psm_fixed_tip, psm_q_k, K, dt);

psm_q_k = psm_q_kp1;

error_pos = [error_pos x_d(1:3,4)-psm_fixed_tip.pos];
error_ori = [error_ori AngleDist(x_d(1:3,1:3),psm_fixed_tip.ori)];

if mod(i,100) == 0
    RunAnimation(mtm_tip_k.pos, mtm_tip_k.ori, psm_fixed_tip.pos, psm_fixed_tip.ori);
end
% RunAnimation(mtm_x(1:3,4,i), mtm_x(1:3,1:3,i), psm_x_dsr(1:3,4,i), psm_x_dsr(1:3,1:3,i));
end

%%
% X = [0;0;0];
% Y = [0;0;0];
% Z = [0;0;0];
% U = [1;0;0];
% V = [0;1;0];
% W = [0;0;1];
% quiver3(X,Y,Z,U,V,W)
% hold on
% origin_pos = [0.5;0.5;0.5];
% theta = pi/4;
% frame_ori = [cos(theta) -sin(theta) 0;
%              sin(theta) cos(theta)  0;
%              0          0           1];
% DrawFrame(origin_pos, frame_ori)