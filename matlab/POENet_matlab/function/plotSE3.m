function plotSE3(logSE3)
% logSE3 : (N,6) matrix of se3 vectors
lineWidth = 0.2;
arrowLength = 0.02;

nData = size(logSE3,1);
for i=1:nData
    SE3 = expm(ToMatrix(logSE3(i,:)'));
    x = SE3(1,4);
    y = SE3(2,4);
    z = SE3(3,4);
    xaxis = arrowLength*SE3(1:3,1);
    yaxis = arrowLength*SE3(1:3,2);
    zaxis = arrowLength*SE3(1:3,3);
    quiver3(x,y,z,xaxis(1),xaxis(2),xaxis(3),'r','LineWidth',lineWidth)
    hold on
    quiver3(x,y,z,yaxis(1),yaxis(2),yaxis(3),'g','LineWidth',lineWidth)
    quiver3(x,y,z,zaxis(1),zaxis(2),zaxis(3),'b','LineWidth',lineWidth)
end
hold off


end