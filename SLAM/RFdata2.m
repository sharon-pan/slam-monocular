function RFdata = RFdata2
% RF data 2
% three line sagement

% xypoint=[0.08	0.857
% 0.179	0.839
% 0.2	0.8
% 0.232	0.745
% 0.285	0.784
% 0.291	0.704
% 0.326	0.672
% 0.564	0.915
% 0.619	0.854
% 0.631	0.804
% 0.701	0.807
% 0.701	0.754
% 0.742	0.731
% 0.792	0.695
% 0.812	0.637
% 0.734	0.364
% 0.774	0.326
% 0.824	0.332
% 0.818	0.273
% 0.883	0.221
% 0.839	0.203
% 0.877	0.153
% 0.927	0.139];

% the data below is the original data deleted a few points
% xypoint=[0.08	0.857
% 0.179	0.839
% 0.2	0.8
% 0.232	0.745
% 0.564	0.915
% 0.619	0.854
% 0.631	0.804
% 0.701	0.807
% 0.701	0.754
% 0.742	0.731
% 0.792	0.695
% 0.812	0.637
% 0.734	0.364
% 0.774	0.326
% 0.824	0.332
% 0.818	0.273
% 0.883	0.221
% 0.839	0.203
% 0.877	0.153
% 0.927	0.139];

% the data below is used to test split and merge
xypoint=[0.7	0.427
0.646	0.406
0.641	0.331
0.582	0.28
0.553	0.197
0.641	0.152
0.743	0.108
0.703	0.106
0.742	0.042
0.833	0.029
0.903	-0.013
0.857	-0.08
0.823	-0.147
0.8	-0.2
0.761	-0.254
0.913	-0.486
0.876	-0.547
0.855	-0.606
0.832	-0.66
0.785	-0.71
0.582	-0.633
0.55	-0.676
0.521	-0.716
0.489	-0.788
0.416	-0.831];

r = zeros(size(xypoint,1),1);
theta = r;

for i = 1:size(xypoint,1)
    r(i) = sqrt(xypoint(i,1)^2 + xypoint(i,2)^2);
    theta(i) = atan2(xypoint(i,2),xypoint(i,1));
end

theta = theta - mean(theta);

RFdata = [theta*180/pi, r];

end

