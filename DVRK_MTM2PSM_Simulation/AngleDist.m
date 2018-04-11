function [ theta ] = AngleDist( R_dsr, R_act )
    Re = R_act' * R_dsr;
    costheta = 0.5 * (Re(1,1) + Re(2,2) + Re(3,3)-1);
    if costheta >= 1
        theta = 0;
    elseif costheta <= -1
        theta = pi;
    else
        theta = acos(costheta);
    end
end

