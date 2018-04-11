function OriError = ComputeOriError (R_dsr, R_act)
    Re = R_act' * R_dsr;
    e = 0.5 * [Re(3,2)-Re(2,3); Re(1,3)-Re(3,1); Re(2,1)-Re(1,2)];
    OriError = R_act *e;
end