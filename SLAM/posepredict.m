function [xp_kp1, Q_k] = posepredict(xp_k,omega_L,omega_R,T,R,L)
% use encoder information to predict pose at time k+1
% xp = [x, y, phi]



xp_kp1 = [xp_k(1) + 0.5*T*R*(omega_L + omega_R)*cos(xp_k(3));
            xp_k(2) + 0.5*T*R*(omega_L + omega_R)*sin(xp_k(3));
            xp_k(3) + T*R/L*(omega_R - omega_L)];
% Error parameter
delta = 0.01;
alpha = 0.01;


Q_k = [delta^2*omega_R^2 + alpha, 0;
        0, delta^2*omega_L^2 + alpha];

end