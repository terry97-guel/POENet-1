clear all
close all
clc

addpath('function')

%% data

L = 1; % initial length
s = (0:0.01:L)'; % points on the line (just for plot, not data)
round_end = (-1:0.01:1)*2*pi; % 
round_end(round_end==0) = 1e-6; % to avoid INF in the "following line"
a = L ./ sqrt(2/pi*round_end); % this is the "following line"
t = s ./ a; % time
angle = 0.5*pi*t(end,:).^2; % end-effector orientation ("q")

logSE3 = zeros(length(round_end), 6);
z_axis = ToMatrix([0;0;1]); % rotational axis (z-axis)

for i = 1:length(round_end)
    SE3 = eye(4);
    % orientation
    SE3(1:3,1:3) = expm(z_axis * angle(i));
    % position
    SE3(1,4) = a(i) .* fresnelc(t(end,i));
    SE3(2,4) = a(i) .* fresnels(t(end,i));
    % Instead of 4x4 SE3 matrix, save 6-dim log(SE3) vector.
    logSE3(i,:) = ToVector(logm(SE3))'; % 4x4 se3 matrix to 6x1 se3 vector
end
dlmwrite('synthetic_euler_spiral.txt',[angle' logSE3],' ')

%% plot

x = a .* fresnelc(t);
y = a .* fresnels(t);
figure(10)
plot(s,zeros(size(s)))
hold on
plot(x, y)
angle = 0.5*pi*t(end,:).^2;
quiver(x(end,:),y(end,:),cos(angle),sin(angle),0.1)
quiver(x(end,:),y(end,:),-sin(angle),cos(angle),0.1)
axis equal
axis tight
hold off
