% main loop
clear; close all;

% Robot parameter
T = 0.032;  % time interval 0.001 sec
R = 0.111;  %[m] wheel radius 2.5 cm
L = 0.381;  %[m] wheel distance 5 cm

xp_k_k = [0, 0, 0]'; % car pose, [x, y, phi(rad)]'
xm_k_k = [];         % line recorded in global map [p, alpha, p2, alpha2, ...]'
xm_LineParameterGR_k_k = []; % another information of the lines in xm_k_k
% LineParameterGlobal = [p, alpha, var_p, var_alpha, cov_pal, x1, x2, var_r, var_psi, cov_rpsi]
P_k_k = zeros(3+size(xm_k_k,1),3+size(xm_k_k,1));

xp_history = xp_k_k;
fig = figure('Position',[1550 50 1000 500]);
sub1 = subplot(1,2,1);
sub2 = subplot(1,2,2);

load('SLAM_data.mat','k_max');

run = true; k = 1;
while run
    % update the variable name
    xp_km_km = xp_k_k; % column vector, [x, y, phi(rad)]'
    xm_km_km = xm_k_k; % column vector, [p, alpha, p2, alpha2, ...]'
    xm_LineParameterGR_km_km = xm_LineParameterGR_k_k; 
    P_km_km = P_k_k;   % size[3+size(xm_k_k,1),3+size(xm_k_k,1)]
    
    
    % read encoder data of time k, omega_L/R is scalar
    [omega_L, omega_R] = readEncoder(k);
    % read Ranger finder data of time k
    RFdata = readRangerFinder(k);
    % RFdata = [theta(deg), r];
    
    % EKF prediction step, including the pose prediction by encoder
    [x_k_km,P_k_km] = EKFpredict(xp_km_km,omega_L,omega_R,xm_km_km,P_km_km,...
    T,R,L);
    
    % Convert RFdata to Line parameter
    clf(fig);
    subplot(1,2,1);
    ob_LineParameterR = RF2RLP(RFdata); % ob : observed Line Parameter
    % LineParameterR = [r, psi, var_r, var_psi, cov_rpsi, x1, x2]
    
    % Line association
    [zM_LPG_k,zM_LPR_k,zN_LPG_k,zN_LPR_k] = LineAssociation(...
    ob_LineParameterR,xm_LineParameterGR_km_km,x_k_km,P_k_km);
    % zM/N_LP_k = [p, alpha, var_p, var_alpha, cov_pal, x1, x2]
    % zM : the line parameter observed by RF and sorted in the order as xm
    % zN : the new line parameter observed by RF, waiting to be added to
    % the map
    
    % Correction step for already observed features
    [x_star_k_k, P_k_k,xm_LineParameterGR_k_k]...
    = EKFupdate(x_k_km,P_k_km,zM_LPG_k,zM_LPR_k,zN_LPG_k,zN_LPR_k);
    xp_k_k = x_star_k_k(1:3);
    xm_k_k = x_star_k_k(4:end);
    
    k = k + 1; 
    % plot the current map and robot trajectory
    xp_history = [xp_history, xp_k_k];
    theta_body = linspace(0,2*pi,80);
    x_body = xp_history(1,end) + 0.5 * cos(theta_body);
    y_body = xp_history(2,end) + 0.5 * sin(theta_body);
    figure(fig); subplot(1,2,2);
    plot(xp_history(1,:),xp_history(2,:),'-.k'); hold on;
    plot(x_body,y_body,'-b','LineWidth',2);
    plot([xp_history(1,end), xp_history(1,end)+cos(xp_history(3,end))],...
        [xp_history(2,end), xp_history(2,end)+sin(xp_history(3,end))],'-r',...
        'LineWidth',2);
    for ii = 1:size(xm_LineParameterGR_k_k,1)
    x = linspace(xm_LineParameterGR_k_k(ii,6), xm_LineParameterGR_k_k(ii,7),100);
    y = (xm_LineParameterGR_k_k(ii,1) - cos(xm_LineParameterGR_k_k(ii,2))*x)/sin(xm_LineParameterGR_k_k(ii,2));
    plot(x,y,'-k','LineWidth',2);  hold on
    end
    axis equal; grid on; axis([-5 25 -15 10])
    title('Global Map','FontSize',14);
    drawnow;
    
    
    
    if k >= k_max
        run = false;
    end
    
end
