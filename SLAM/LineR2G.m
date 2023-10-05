function LineParameterGlobal = LineR2G(LineParameterRob,xp_k_km,Ppp_k_km)
% convert the observation from robot frame to global frame
% dealing with only one line once.
% LineParameterRob = [r, psi, var_r, var_psi, cov_rpsi, x1, x2]
% LineParameterGlobal = [p, alpha, var_p, var_alpha, cov_pal, x1, x2]

r = LineParameterRob(1); psi = LineParameterRob(2);
x1_r = LineParameterRob(6); x2_r = LineParameterRob(7);
x = xp_k_km(1); y = xp_k_km(2); phi = xp_k_km(3);
R = [LineParameterRob(3), LineParameterRob(5);
    LineParameterRob(5), LineParameterRob(4)];

D = r - x*sin(psi + phi) + y*cos(psi + phi);
p_hat = abs(D);
alpha_hat = psi + (phi - pi/2) - (-0.5*sign(D) + 0.5)*pi;

d = sign(D);
o = -d*x*cos(psi + phi) - d*y*sin(psi + phi);

Gx = [ - d * sin(psi + phi) , 0;
         d * cos(psi + phi) , 0;
         o                  , 1]';
Gz = [d o; 0 1];

S = Gx*Ppp_k_km*Gx' + Gz*R*Gz';

y1_r = (r - cos(psi)*x1_r)/sin(psi);
x1_g = x1_r * cos(phi - pi/2) - y1_r*sin(phi - pi/2);
y2_r = (r - cos(psi)*x2_r)/sin(psi);
x2_g = x2_r * cos(phi - pi/2) - y2_r*sin(phi - pi/2);

LineParameterGlobal = [p_hat, alpha_hat, S(1,1), S(2,2), S(1,2), x1_g, x2_g];

end