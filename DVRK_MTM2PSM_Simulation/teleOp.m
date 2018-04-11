classdef teleOp < handle
  properties(Access = public)

    publisher;
    jointStateMsg;
    tStart = tic;
    dt = 0.001;
    mtm_q_initial = [-0.000346807743412636;-0.0200600850493284;-0.0143117565895542;-0.719724490423581;1.56913839839585;0;2.42180221142322];
    psm_q_initial = [0.0174532925199433;-0.0349065850398866;0.180000000000000;0;-0.0523598775598299;0.00872664625997165];
    flag = 0;
    % add your code
    scale = 0.5; %translation scale from mtmFixedTip to psmTip
    k1  = 500;
    k2  = 500;
    K;
    mtm_tip_k;
    psm_fixed_tip_k_pos;
    psm_q_k;
  end

    methods(Access = public)
        
        function obj = teleOp(publisher,jointStateMsg)  % Constructor

            if (nargin > 1)
                obj.publisher = publisher;
                obj.jointStateMsg = jointStateMsg;
            end
            % add your code
        [mtm_tip_initial,~,~] = FK(MTMModel( ), obj.mtm_q_initial);  % Compute the tip information at MTM in the first sampling
        obj.mtm_tip_k = mtm_tip_initial;                        % mtm_tip_k denotes the "old" MTM tip information. "k" means "old"; "kp1", i.e., k+1, means "new". 

        [~, ~, psm_fixed_tip_k] = FK(PSMModel( ),obj.psm_q_initial); % Compute the fixed tip information at PSM in the first sampling
        obj.psm_fixed_tip_k_pos = psm_fixed_tip_k.pos;               % PSM fixed tip "old" position
        obj.psm_q_k = obj.psm_q_initial;                                 % PSM "old" joint angle space(vector)
         obj.K  = diag([obj.k1,obj.k1,obj.k1,obj.k2,obj.k2,obj.k2],0); % Gain matrix
        end


        
        function  [psm_q,tracking_err] = run(obj, mtm_q_realtime)
            obj.flag = obj.flag + 1;



            error_pos = [];                                          % Position error for verifying inverse kinematics
            error_ori = [];                                          % Orientation error for verifying inverse kinematics

            [mtm_tip_kp1, ~, ~] = FK(MTMModel( ), mtm_q_realtime);     % Compute the tip information at MTM, considered as "new" info
            %  mtm_tip_pos = [mtm_tip_pos mtm_tip.pos];              % Combine the
                                                                     % position information, used for verifying forward kinematics
            % % 
            [ x_d, dx_d ] = TeleopRelationship( obj.mtm_tip_k, mtm_tip_kp1, obj.psm_fixed_tip_k_pos, obj.dt ,obj.scale); % Compute inverse kinematics inputes.
            obj.mtm_tip_k = mtm_tip_kp1;                                 % Save "old" MTM tip info for next loop
            obj.psm_fixed_tip_k_pos = x_d(1:3,4);                        % 

            % 
            % x_d  = psm_x_dsr(:,:,i);
            % dx_d = psm_xdot_dsr(:,i);

            [~, psm_J, psm_fixed_tip] = FK(PSMModel( ), obj.psm_q_k); % psm_tip->joint6,psm_fixed_tip->joint7

            psm_q_kp1 = IK( x_d, dx_d, psm_J, psm_fixed_tip, obj.psm_q_k, obj.K, obj.dt);

            obj.psm_q_k = psm_q_kp1;
            psm_q = psm_q_kp1;
            
            error_pos = x_d(1:3,4)-psm_fixed_tip.pos;
            error_ori = AngleDist(x_d(1:3,1:3),psm_fixed_tip.ori);
            tracking_err = [error_pos;error_ori];

            if mod(obj.flag,10) == 0
                RunAnimation(mtm_tip_kp1.pos, mtm_tip_kp1.ori, psm_fixed_tip.pos, psm_fixed_tip.ori);
            end
        end    
        
        function  callback_update_mtm_q(obj,q)
            obj.jointStateMsg.Position = obj.run(q);
            tElapsed = toc(obj.tStart);
            if (tElapsed > 0.033)
                obj.tStart = tic;
                obj.publisher.send(obj.jointStateMsg);
            end    
        end

    end
end
