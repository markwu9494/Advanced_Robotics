function [ psm_q_kp1 ] = IK(  x_d, dx_d, J, psm_tip, psm_q_k, K, dt)

x_pos_error = x_d(1:3,4) - psm_tip.pos;
x_ori_error = ComputeOriError (x_d(1:3,1:3), psm_tip.ori);
x_error     = [x_pos_error; x_ori_error];

dq    = SVD_PesudoInverse(J) * (dx_d + K * x_error);
psm_q_kp1 = psm_q_k + dq * dt;
end

