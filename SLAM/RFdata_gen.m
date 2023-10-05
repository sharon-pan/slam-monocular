% gen Ranger finder data
% unit meter
% given the (a,b) of y=ax+b
% gen the data of range finder [theta, r] in the unit of [deg, m]
clear; close all;

a = 1;
b = 0.5;

theta = -30:1:30;
scale = pi/180;
rangeMax = 1; % 1 meter
leftLimtX = rangeMax * cos((90 - min(theta))*pi/180);
leftLimtY = rangeMax * sin((90 - min(theta))*pi/180);
rightLimtX = rangeMax * cos((90 - max(theta))*pi/180);
rightLimtY = rangeMax * sin((90 - max(theta))*pi/180);

theta = theta + 90;
r = abs(b./(sin(theta*scale)-a.*cos(theta*scale)));
r_noised = r + 0.025*randn(size(r));

x = r.*cos(theta*scale); y = r.*sin(theta*scale);
x_noised = r_noised.*cos(theta*scale);
y_noised = r_noised.*sin(theta*scale);

figure;
plot(x,y,'x-','MarkerSize',2); hold on;
plot(x_noised,y_noised,'o','MarkerSize',2)
plot(0,0,'xk','MarkerSize',12,'LineWidth',4)
plot([0 leftLimtX],[0 leftLimtY],'k-','LineWidth',1)
plot([0 rightLimtX],[0 rightLimtY],'k-','LineWidth',1)
grid on; axis equal; 
legend('real point/line','measured point')

theta = theta - 90; % return the data to [-x, x]
RFdata = [theta', r_noised'];

