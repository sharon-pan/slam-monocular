function LineParameter = RF2RLP(RFdata)
% RF2RLP = Range Finder to Robot frame Line Parameter
% gen RFdata RFdata = [theta', r'];
% clear; close all
% RFdata = RFdata2;

%% transform the polar coordinate to xy coord.
scale = pi / 180 ; 
x_scan = RFdata(:,2).*cos(RFdata(:,1)*scale);
y_scan = RFdata(:,2).*sin(RFdata(:,1)*scale);
point_scan = [x_scan, y_scan];
%% delete the point that is too far
r_th = 10; % 10 meter
ind=find(RFdata(:,2)>r_th);
if ~isempty(ind)
    point_scan(ind,:) = []; % delete
end
%% Split the group of points if one point is far away from its neighbor
dis_th = 0.5; % 20 cm
group_num = ones(size(point_scan,1),1);
for i = 2:size(point_scan,1)
    dis = (point_scan(i-1,1) - point_scan(i,1))^2 ...
        + (point_scan(i-1,2) - point_scan(i,2))^2;
    if dis > dis_th^2
        group_num(i) = group_num(i-1)+1;
    else
        group_num(i) = group_num(i-1);
    end
end
%% if the number of point in that group is less than Nmin
Nmin = 7; 
for i = 1:max(group_num)
    ind = find(group_num == i);
    if length(ind) < Nmin
        point_scan(ind,:) = [];
        group_num(ind,:) = [];
        disp(['delete group num ',num2str(i)])
    end
end
%% Rearrange the group number, avoid the jump in group number
ind_jump = find(diff(group_num)~=0) + 1 ;
group_num(1) = 1;
for i = 2:length(group_num)
    if sum(i == ind_jump) == 1
        group_num(i) = group_num(i-1) + 1;
    else
        group_num(i) = group_num(i-1);
    end
end
%% Split and Merge --- Split
split_th = 0.5;% 10 cm
tochecksplit = true;
split_point = zeros(length(group_num),1);
i = 1;
while tochecksplit
    dis2line = [];
    ind_p1 = find(group_num == i,1,'first'); % index of the first point in the group
    ind_p2 = find(group_num == i,1,'last'); % index of the last point in the group
    dis2line = point2line(point_scan(ind_p1,1),point_scan(ind_p1,2),...
        point_scan(ind_p2,1),point_scan(ind_p2,2),...
        point_scan(group_num==i,1),point_scan(group_num==i,2));
     ind_dis2line = ind_p1:1:ind_p2;
    if max(dis2line) > split_th
        [~,ind_max_dis2line] = max(dis2line);
        ind_split = ind_dis2line(ind_max_dis2line);
        split_point(ind_split) = 1;
        group_num(ind_p1:ind_split,1) = max(group_num) + 1;
        group_num(ind_split+1:ind_p2,1) = max(group_num) + 1;
    end
    
    i = i + 1;
    
    if i > max(group_num)
        tochecksplit = false;
    end
end
%% Rearrange the group number, avoid the jump in group number
ind_jump = find(diff(group_num)~=0) + 1 ;
group_num(1) = 1;
for i = 2:length(group_num)
    if sum(i == ind_jump) == 1
        group_num(i) = group_num(i-1) + 1;
    else
        group_num(i) = group_num(i-1);
    end
end
%% Fitting : gen line parameter
LineParameter = zeros(max(group_num),7);
% = [r, psi, var_r, var_psi, cov_rpsi, x1, x2]
for i = 1 : max(group_num)
    ind_p1 = find(group_num == i,1,'first'); % index of the first point in the group
    ind_p2 = find(group_num == i,1,'last'); % index of the last point in the group
    if ind_p1 ~= 1
        if split_point(ind_p1 - 1,1) == 1
            ind_p1 = ind_p1 - 1;
        end
    end
    y =   point_scan(ind_p1:ind_p2,2) ;
    U = [ point_scan(ind_p1:ind_p2,1), ones(ind_p2 - ind_p1 + 1,1)];
    theta_hat = (U'*U)\U'*y;
    k_hat = theta_hat(1); 
    c_hat = theta_hat(2);
    r_hat = c_hat*sign(c_hat)/sqrt(k_hat^2 + 1);
    psi_hat = atan2(sign(c_hat)/sqrt(k_hat^2 + 1), -k_hat*sign(c_hat)/sqrt(k_hat^2 + 1));
    y_hat = k_hat * point_scan(ind_p1:ind_p2,1) + c_hat;
    var_y = sum( (y - y_hat).^2 ) / (length(y) - 1);
    Z = var_y * (U'*U\[1,0;0,1]);
    % Z = [ var_k, cov_kc; cov_ck, var_c];
    Krk = - c_hat * k_hat * sign(c_hat) / ((k_hat^2 + 1)^(3/2));
    Krc = sign(c_hat)/sqrt(k_hat^2 + 1);
    Kpsik = sign(c_hat)/(k_hat^2 + 1);
    var_psi = Kpsik^2 * Z(1,1);
    var_r = Krk^2 * Z(1,1) + Krc^2 * Z(2,2) + 2*Krk*Krc*Z(1,2);
    cov_rpsi = Krk*Kpsik*Z(1,1) + Krc*Kpsik*Z(2,1);
%     R = [var_r, cov_rpsi;
%          cov_rpsi, var_psi];
    x1 = min(point_scan(ind_p1,1), point_scan(ind_p2,1));
    x2 = max(point_scan(ind_p1,1), point_scan(ind_p2,1));
    LineParameter(i,:) = [ r_hat, psi_hat, var_r, var_psi, cov_rpsi, ...
        x1, x2];
    
end

%% plot out
% figure; 
plot(x_scan,y_scan,'ob'); hold on;
plot(point_scan(:,1), point_scan(:,2), '-xk')
color_map = jet(max(group_num));
for ii = 1:size(point_scan,1)
    plot(point_scan(ii,1), point_scan(ii,2),'p','Color',color_map(group_num(ii),:));
end
axis equal; axis([-10 10 -5 15]);
for ii = 1:size(LineParameter,1)
    x = LineParameter(ii,6) : 0.001 : LineParameter(ii,7);
    y = (LineParameter(ii,1) - cos(LineParameter(ii,2))*x)/sin(LineParameter(ii,2));
    plot(x,y,'-','Color',color_map(ii,:),'LineWidth',2);  hold on
end
title('RengeFinder data at time k','FontSize',14);
end

    
    