function [ x_d, dx_d ] = TeleopRelationship( mtm_tip_k, mtm_tip_kp1, psm_fixed_tip_k_pos, dt ,translationScale)
% T    = [-1  0 0 0 ;  %% from MTM base frame to PSM base frame
%          0 -1 0 0 ;
%          0  0 1 0;
%          0  0 0 1];
% 
% % R = [0    -1     0;
% %      1     0     0;
% %      0     0     1];    
% % T = [R    zeros(3,1) ;
% %      zeros(1,3)         1 ;            
% %        ];
%    
% HT_k = [mtm_tip_k.ori  mtm_tip_k.pos ;
%             zeros(1,3)                1 ;            
%        ];
% HT_kp1 = [mtm_tip_kp1.ori  mtm_tip_kp1.pos ;
%             zeros(1,3)                1 ;            
%        ];
% 
% HT_k_new   = T * HT_k;
% HT_kp1_new = T * HT_kp1;
% 
% mtm_tip_k.pos = HT_k_new(1:3,4);
% mtm_tip_k.ori = HT_k_new(1:3,1:3);
% 
% mtm_tip_kp1.pos = HT_kp1_new(1:3,4);
% mtm_tip_kp1.ori = HT_kp1_new(1:3,1:3);

% x_d = [mtm_tip_kp1.ori  mtm_tip_kp1.pos ;
%             zeros(1,3)                1 ;            
%        ];
% dx_d = [(mtm_tip_kp1.pos-mtm_tip_k.pos)/dt;ComputeOriError(mtm_tip_kp1.ori, mtm_tip_k.ori)/dt];


R   = [-1  0 0 ;  %% from MTM base frame to PSM base frame
        0 -1 0 ;
        0  0 1 ;
       ];


psm_tip_k.ori = R * mtm_tip_k.ori;
psm_tip_kp1.ori = R * mtm_tip_kp1.ori;

psm_fixed_tip_kp1_pos = psm_fixed_tip_k_pos + R * translationScale*(mtm_tip_kp1.pos-mtm_tip_k.pos);


x_d = [psm_tip_kp1.ori  psm_fixed_tip_kp1_pos ;
            zeros(1,3)                1 ;            
       ];
dx_d = [ R * (mtm_tip_kp1.pos-mtm_tip_k.pos)/dt;ComputeOriError(psm_tip_kp1.ori, psm_tip_k.ori)/dt];


end

