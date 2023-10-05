function [x_k_km,P_k_km] = EKFpredict(xp_km_km,omega_L,omega_R,xm_km_km,P_km_km,...
    T,R,L)
% m for -1

% pose prediction by encoder
[xp_k_km, Q_k] = posepredict(xp_km_km,omega_L,omega_R,T,R,L);
% xp_k_km is a 3*1 row vector
xm_k_km = xm_km_km; % xm is a m*1 row vector
x_k_km = [xp_k_km; xm_k_km];

Fx = [  1, 0, -0.5*T*R*(omega_R+omega_L)*sin(xp_km_km(3));
        0, 1,  0.5*T*R*(omega_R+omega_L)*cos(xp_km_km(3));
        0, 0, 1];
Fu = 0.5*T*R*[  cos(xp_km_km(3)), cos(xp_km_km(3));
                sin(xp_km_km(3)), sin(xp_km_km(3));
                2/L             , 2/L];

Ppp_km_km = P_km_km(1:3,1:3);
Ppm_km_km = P_km_km(1:3,4:end);
% Pmp_km_km = P_km_km(4:end,1:3);
Pmm_km_km = P_km_km(4:end,4:end); % not sure, what is the form of P??

Ppp_k_km = Fx*Ppp_km_km*Fx' + Fu*Q_k*Fu';
if size(Ppm_km_km,1) == 0
    Ppm_k_km = [];
else
    Ppm_k_km = Fx*Ppm_km_km;
end
Pmm_k_km = Pmm_km_km;

P_k_km = [Ppp_k_km, Ppm_k_km;
        Ppm_k_km', Pmm_k_km];

end