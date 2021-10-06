function plot_screw_theta_with_pose(outputPose,M_se3, screw, theta, len, viewAxis)
    
    fig = figure();
    set(fig, 'OuterPosition', [0,  0, 1600, 900])
    plotSE3(outputPose)
    hold on
    axis equal
    
    nData = size(theta,1);
    original_screw = screw;
    
    nJoint = size(screw,2);
    color = zeros(nJoint,3);
    color(:,1) = linspace(1,0,nJoint);
    color(:,3) = linspace(0,1,nJoint);
    
    for k=1:nJoint
        w = screw(1:3,k);
        v = screw(4:6,k);
        q_zero(:,k) = skew(w) * v;
    end 
    
    for j=1:nData
        screw = original_screw;
        poe = eye(4);
        
        POE = eye(4);
        for i=1:nJoint
            q_temp = POE*[q_zero(:,i);1];
            q_link(:,i) = q_temp(1:3);
            POE = POE * expm(ToMatrix(original_screw(:,i))*theta(j,i));
        end
        end_effector = POE*expm(ToMatrix(M_se3));
        q_link(:,nJoint+1) = end_effector(1:3,4);
        
        for i=1:nJoint
            w = screw(1:3,i);
            q = q_link(:,i);
            t = -len:0.1:len;
            jointaxis = q + w * t;
            x = jointaxis(1,:);
            y = jointaxis(2,:);
            z = jointaxis(3,:);
            if j == 1
                p(i) = plot3(x, y, z, 'LineWidth', 1, 'Color', color(i,:), 'LineWidth', 2);
                hold on;
                view(viewAxis)
%                 p(i).XDataSource = 'x';
%                 p(i).YDataSource = 'y';
%                 p(i).ZDataSource = 'z';
            else
                p(i).XData = x;
                p(i).YData = y;
                p(i).ZData = z;
            end
            if i < nJoint
                poe = poe * expm(ToMatrix(original_screw(:,i))*theta(j,i));
                screw(:,i+1) = largeAdjoint(poe)*screw(:,i+1);
            end
        end
        
        qx = q_link(1,:);
        qy = q_link(2,:);
        qz = q_link(3,:);
        end_qx = q_link(1,nJoint+1);
        end_qy = q_link(2,nJoint+1);
        end_qz = q_link(3,nJoint+1);
        if j==1
            link = plot3(qx, qy, qz, 'Color','black');
%             link.XDataSource = 'qx';
%             link.YDataSource = 'qy';
%             link.ZDataSource = 'qz';
            hold on;
            end_q = plot3(end_qx, end_qy, end_qz,'o','MarkerFaceColor','red');
%             end_q.XDataSource = 'end_qx';
%             end_q.YDataSource = 'end_qy';
%             end_q.ZDataSource = 'end_qz';
            hold on;
        else
%             refreshdata(link)
%             refreshdata(end_q)
%             drawnow
            link.XData = qx;
            link.YData = qy;
            link.ZData = qz;
            end_q.XData = end_qx;
            end_q.YData = end_qy;
            end_q.ZData = end_qz;
        end
        x_range(j,:) = get(gca,'XLim');
        y_range(j,:) = get(gca,'YLim');
        z_range(j,:) = get(gca,'ZLim');
    end
    
    hold off

    plotSE3(outputPose)
    axis equal
    hold on

    videoObj = VideoWriter(int2str(nJoint));
    videoObj.FrameRate = 30;
    open(videoObj);
    
    for j=1:nData
        screw = original_screw;
        poe = eye(4);
        POE = eye(4);
        for i=1:nJoint
            q_temp = POE*[q_zero(:,i);1];
            q_link(:,i) = q_temp(1:3);
            POE = POE * expm(ToMatrix(original_screw(:,i))*theta(j,i));
        end
        end_effector = POE*expm(ToMatrix(M_se3));
        q_link(:,nJoint+1) = end_effector(1:3,4);
        
        for i=1:nJoint
            w = screw(1:3,i);
            q = q_link(:,i);
            t = -len:0.1:len;
            jointaxis = q + w * t;
            x = jointaxis(1,:);
            y = jointaxis(2,:);
            z = jointaxis(3,:);
            if j == 1
                p(i) = plot3(x, y, z, 'LineWidth', 1, 'Color', color(i,:), 'LineWidth', 2);
                hold on;
                view(viewAxis)
%                 p(i).XDataSource = 'x';
%                 p(i).YDataSource = 'y';
%                 p(i).ZDataSource = 'z';
            else
                p(i).XData = x;
                p(i).YData = y;
                p(i).ZData = z;
            end
            if i < nJoint
                poe = poe * expm(ToMatrix(original_screw(:,i))*theta(j,i));
                screw(:,i+1) = largeAdjoint(poe)*screw(:,i+1);
            end
        end
        
        qx = q_link(1,:);
        qy = q_link(2,:);
        qz = q_link(3,:);
        end_qx = q_link(1,nJoint+1);
        end_qy = q_link(2,nJoint+1);
        end_qz = q_link(3,nJoint+1);
        if j==1
            link = plot3(qx, qy, qz, 'Color','black');
%             link.XDataSource = 'qx';
%             link.YDataSource = 'qy';
%             link.ZDataSource = 'qz';
            hold on;
            end_q = plot3(end_qx, end_qy, end_qz,'o','MarkerFaceColor','red');
%             end_q.XDataSource = 'end_qx';
%             end_q.YDataSource = 'end_qy';
%             end_q.ZDataSource = 'end_qz';
            hold on;
        else
%             refreshdata(link)
%             refreshdata(end_q)
%             drawnow
            link.XData = qx;
            link.YData = qy;
            link.ZData = qz;
            end_q.XData = end_qx;
            end_q.YData = end_qy;
            end_q.ZData = end_qz;
        end
        xlim([min(x_range(:,1)) max(x_range(:,2))])
        ylim([min(y_range(:,1)) max(y_range(:,2))])
        zlim([min(z_range(:,1)) max(z_range(:,2))])
        writeVideo(videoObj,getframe)
    end
    close(videoObj);
end