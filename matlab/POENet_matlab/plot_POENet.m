clear all
close all
clc

addpath('function')
addpath('data')

%% load data
jointTwist = dlmread('data/euler_spiral/jointTwist.txt')';
jointAngle = dlmread('data/euler_spiral/jointAngle.txt');
M_se3 = dlmread('data/euler_spiral/M_se3.txt');
targetPose = dlmread('data/euler_spiral/targetPose.txt');
outputPose = dlmread('data/euler_spiral/outputPose.txt');
nJoint = size(jointTwist,2);
%% plot output of POENet and target
figure(1);
plotSE3(outputPose)
axis equal
hold on;
plotSE3(targetPose)
axis equal
%% make video
fig = figure(2);
set(fig, 'OuterPosition', [0,  0, 1600, 900])
plot_screw_theta_with_pose(outputPose,M_se3, jointTwist, jointAngle, 0.2, [1 1 1])
hold off
%% plot q-theta map
figure(3)
plot(jointAngle)



