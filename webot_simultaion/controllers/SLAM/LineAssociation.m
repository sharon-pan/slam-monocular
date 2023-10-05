function [zM_LPG_k,zM_LPR_k,zN_LPG_k,zN_LPR_k] = LineAssociation(...
    ob_LineParameterR,xm_LineParameterGR_km_km,x_k_km,P_k_km)
% figure out whether some of observed line is the same withe the line in xm
% ob_LineParameterR = [r, psi, var_r, var_psi, cov_rpsi, x1, x2]
% xm_LineParameterGR_km_km = [p, alpha, var_p, var_alpha, cov_pal, x1, x2, var_r, var_psi, cov_rpsi]



if size(xm_LineParameterGR_km_km,1) == 0
    % first round
    zM_LPG_k = [];
    zM_LPR_k = [];
    
    % transfer the line parameter from robot frame to global frame
    xp_k_km = x_k_km(1:3);
    Ppp_k_km = P_k_km(1:3,1:3);
    ob_LineParameterG = zeros(size(ob_LineParameterR,1),7);
    for i = 1:size(ob_LineParameterR,1)
        ob_LineParameterG(i,:) = LineR2G(ob_LineParameterR(i,:),xp_k_km,Ppp_k_km);
    end
    % LineParameterGlobal = [p, alpha, var_p, var_alpha, cov_pal, x1, x2]
    zN_LPG_k = ob_LineParameterG;
    zN_LPR_k = ob_LineParameterR;
else
    % second round and so on
    
    zM_LPG_k = xm_LineParameterGR_km_km; % the line parameter for update in global and the R covariance matrix elem.
    zM_LPR_k = zeros(size(xm_LineParameterGR_km_km,1),7); % in robot frame
    zM_LPR_k(:,3:5) = xm_LineParameterGR_km_km(:,8:10);
    zN_LPG_k = [];   % the line parameter [p, alpha] for adding new line to map
    zN_LPR_k = [];   % in robot frame [r, psi]
    
    % transfer the line parameter from robot frame to global frame
    xp_k_km = x_k_km(1:3);
    Ppp_k_km = P_k_km(1:3,1:3);
    ob_LineParameterG = zeros(size(ob_LineParameterR,1),7);
    for i = 1:size(ob_LineParameterR,1)
        ob_LineParameterG(i,:) = LineR2G(ob_LineParameterR(i,:),xp_k_km,Ppp_k_km);
    end
    % LineParameterGlobal = [p, alpha, var_p, var_alpha, cov_pal, x1, x2]
    
    d_th = 20000; % the threshold to make sure the lines are the same.
    for i = 1:size(ob_LineParameterG,1)
        for j = 1:size(xm_LineParameterGR_km_km,1) % j number of line in global map

            S = [ob_LineParameterG(i,3), ob_LineParameterG(i,5);
                 ob_LineParameterG(i,5), ob_LineParameterG(i,4)];
            A = [xm_LineParameterGR_km_km(j,1) - ob_LineParameterG(i,1);
                 xm_LineParameterGR_km_km(j,2) - ob_LineParameterG(i,2)];
            d = A'*(S\A); 
            disp(['i = ',num2str(i),', j = ',num2str(j),' d = ',num2str(d)]);
            
            xo1 = ob_LineParameterG(i,6); xo2 = ob_LineParameterG(i,7);
            xm1 = xm_LineParameterGR_km_km(j,6); xm2 = xm_LineParameterGR_km_km(i,7);
            
            if d < d_th %&& ((xo1>xm1 && xo1<xm2) ||(xo2>xm1 && xo2<xm2))
                disp(['observation of line ',num2str(i),' is the same as the line '...
                    ,num2str(j),' in the map']);
                % if the d is small enough and
                % one of the end points of the line observed lies between the end points of the line in map 
                zM_LPG_k(j,1:7) = ob_LineParameterG(i,:);
%                 zM_LPG_k(j,6) = min(xo1,xm1); zM_LPG_k(j,7) = max(xo2, xm2);
                zM_LPG_k(j,6) = 0.5*(xo1+xm1); zM_LPG_k(j,7) = 0.5*(xo2+xm2);
                
                zM_LPR_k(j,:) = ob_LineParameterR(i,:);
                % x1, x2 not update in the robot frame version.
                break;
            end

            if j == size(xm_LineParameterGR_km_km,1)
                % if d < d_th for all line existing now
                zN_LPG_k = [ zN_LPG_k; ob_LineParameterG(i,:)];
                zN_LPR_k = [ zN_LPR_k; ob_LineParameterR(i,:)];
                disp('New line!!');
            end

        end
    end
    
    
end

end
