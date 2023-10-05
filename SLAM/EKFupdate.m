function [x_star_k_k, P_star_k_k,xm_star_LineParameterGR_k_k]...
    = EKFupdate(x_k_km,P_k_km,zM_LPG_k,zM_LPR_k,zN_LPG_k,zN_LPR_k)

% ---------test
% clear; close all;
% x_k_km = [1 2 3 4 5 6 7 8 9]';
% -----------test

% transfer line[p, alpha] which is predicted by EKF into robot frame
M = (size(x_k_km,1)-3)/2;

if M == 0
    x_k_k = x_k_km;
    P_k_k = P_k_km;
else
    % for second and futher run...

    Hx = zeros(2*M, 3+2*M);
    zM_hat = zeros(2*M,1); j = 1;
    xr = x_k_km(1); yr = x_k_km(2); phir = x_k_km(3);
    for i = 4:2:(size(x_k_km,1)-1)
        p_i = x_k_km(i); alphai = x_k_km(i+1); % p_i reffer to p_i in paper
        Ci = p_i - xr*cos(alphai) - yr*sin(alphai);
        rj_hat = abs(Ci);
        psij_hat = alphai - (phir- pi/2) + 0.5*(-sign(Ci)+1)*pi;
        % construct the zM_hat vector
        zM_hat(2*j-1,1) = rj_hat;
        zM_hat(2*j,1) = psij_hat;
        % construct the observation jacobian Hx
        ci = sign(Ci);
        gi = ci*xr*sin(alphai) - ci*yr*cos(alphai);
        Hpj = [-ci*cos(alphai), -ci*sin(alphai), 0;
               0, 0, -1;];
        Hmj = zeros(2,2*M);
        Hmj(1,2*j-1) = ci; Hmj(1,2*j) = gi; Hmj(2,2*j) = 1;
        Hx(2*j-1:2*j,:) = [Hpj, Hmj];
        j = j + 1;
    end

    % construct zM_ob from zM_LPR_k: the line parameter in robot frame got from observation 
    zM_ob = zM_LPR_k(:,1:2)';
    zM_ob = zM_ob(:);

    for i = 1:size(zM_ob)
        % 如果zM_ob中沒有數值，那就把zM_hat的那一項刪掉，避免計算出殘差
        if zM_ob(i)==0
            zM_hat(i) = 0;
        end
    end

    % construct R from the zM_LPR_k, which containt the R element from
    % observation or the dataset(for line already in map but not observed this time.
    Rm = zeros(2*M,2*M);
    for j = 1 : M
        Rm(2*j-1:2*j,2*j-1:2*j) = [zM_LPR_k(j,3), zM_LPR_k(j,5); zM_LPR_k(j,5), zM_LPR_k(j,4)];
    end

    K = P_k_km*Hx'*((Hx*P_k_km*Hx'+Rm)\eye(size(Rm,1)));

    x_k_k = x_k_km + K*(zM_ob - zM_hat);
    P_k_k = P_k_km - K*Hx*P_k_km;
end

% -------------------Correction for new features-------------------------

% construct the jacobian matrices, Gx, Gz
N = size(zN_LPG_k,1);
zN_toadd = zeros(2*N,1);
Gx = zeros(2*N,3);
Gz = zeros(2*N,2*N);
xr = x_k_k(1); yr = x_k_k(2); phir = x_k_k(3);
% zN_LPG_k = [];   % the line parameter [p, alpha] for adding new line to map
% zN_LPR_k = [];   % in robot frame [r, psi]
for i = 1:N
    rf = zN_LPR_k(i,1); psif = zN_LPR_k(i,2); 
    Di = rf - xr * sin(psif + phir) + yr * cos(psif + phir);
    di = sign(Di);
    zN_toadd(2*i-1,1) = abs(Di);
    zN_toadd(2*i,1) = psif + (phir - pi/2) - (-0.5*sign(Di)+0.5)* pi ;
    oi = - di * xr * cos(psif + phir) - di * yr * sin(psif + phir );
    Gxf = [- di * sin(psif + phir), di * cos(psif + phir), oi; 0,0,1];
    Gzf = [di, oi; 0, 1];
    
    Gx(2*i-1:2*i,:) = Gxf;
    Gz(2*i-1:2*i,2*i-1:2*i) = Gzf;
    
end

Rn = zeros(2*N, 2*N);
for i = 1:N
    Rn(2*i-1:2*i,2*i-1:2*i) = [ zN_LPR_k(i,3), zN_LPR_k(i,5); zN_LPR_k(i,5), zN_LPR_k(i,4)];
end

if M ~= 0
    Gx = [Gx, zeros(2*N,2*M)];
end

x_star_k_k = [x_k_k',  zN_toadd']';
P_star_k_k = [P_k_k, P_k_k*Gx'; Gx*P_k_k, Gx*P_k_k*Gx' + Gz*Rn*Gz'];

xm_star_LineParameterGR_k_k = zeros(M+N,10);
% [p, alpha, var_p, var_alpha, cov_pal, x1, x2, var_r, var_psi, cov_rpsi]
xm_star_LineParameterGR_k_k(:,1:2) = reshape(x_star_k_k(4:end,1),2,[])';
if M == 0
    % for first run
    xm_star_LineParameterGR_k_k(M+1:M+N,3:7) = zN_LPG_k(:,3:7);
    xm_star_LineParameterGR_k_k(M+1:M+N,8:10) = zN_LPR_k(:,3:5);
elseif N==0
    xm_star_LineParameterGR_k_k(1:M,3:7) = zM_LPG_k(:,3:7);
    xm_star_LineParameterGR_k_k(1:M,8:10) = zM_LPR_k(:,3:5);
else
    % for second run and so on
    xm_star_LineParameterGR_k_k(1:M,3:7) = zM_LPG_k(:,3:7);
    xm_star_LineParameterGR_k_k(1:M,8:10) = zM_LPR_k(:,3:5);
    xm_star_LineParameterGR_k_k(M+1:M+N,3:7) = zN_LPG_k(:,3:7);
    xm_star_LineParameterGR_k_k(M+1:M+N,8:10) = zN_LPR_k(:,3:5);
end

end