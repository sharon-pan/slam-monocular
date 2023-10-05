function [omega_L, omega_R] = readEncoder(k)
load('SLAM_data.mat','omgL','omgR');
omega_L = omgL(k);
omega_R = omgR(k);
end