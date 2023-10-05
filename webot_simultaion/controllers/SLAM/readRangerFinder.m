function RFdata = readRangerFinder(k)
% RFdata = [theta(deg), r];
load('SLAM_data.mat','LidarDIST','LidarTHETA');
RFdata = [LidarTHETA', LidarDIST(k,:)'];
end