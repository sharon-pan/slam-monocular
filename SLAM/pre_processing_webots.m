% pre processing of raw dataset from webots
clear; close all;
load("dataset.csv");

LidarDIST = dataset(1:end-1,1:180);
LidarTHETA = linspace(180,0,180);

encL = mean(dataset(:,[181,183])')';
encR = mean(dataset(:,[182,184])')';
omgL = diff(encL)./0.032;
omgR = diff(encR)./0.032;

k_max = size(dataset,1);

save('SLAM_data.mat','LidarDIST','LidarTHETA','omgL','omgR','k_max');




