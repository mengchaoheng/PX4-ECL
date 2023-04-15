clear all;
close all;

% add required paths
addpath('../Common');

% set parameters to default values
run('SetParameters.m');

% load test data
load '../TestData/PX4/baro_data.mat';
load '../TestData/PX4/imu_data.mat';
load '../TestData/PX4/mag_data.mat';
param.fusion.enable=exist("../TestData/PX4/gps_data.mat","file");
if param.fusion.enable
    load '../TestData/PX4/gps_data.mat';
else
    gps_data=[];
end


% run the filter replay
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data);

% generate and save output plots
runIdentifier = ' : PX4 data replay ';
folder = strcat('../OutputPlots/PX4');
PlotData(output,folder,runIdentifier,param.fusion.enable);

% save output data
folder = '../OutputData/PX4';
fileName = '../OutputData/PX4/ekf_replay_output.mat';
if ~exist(folder,'dir')
    mkdir(folder);
end
save(fileName,'output');