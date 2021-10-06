function [del_eta, dim_of_null] = find_tangent_del_eta(screws, thetas, dq_dtheta, eig_thld)
    nJoint = length(thetas(1,:));
    nQ = length(dq_dtheta(1,:));
    nData = length(thetas(:,1));
    if any(size(screws) ~= [6, nJoint+1])
        error('Wrong input for find_tangent_del_eta()')
    end
    revoluteBase = NullspaceRevolute(screws(:,1:end-1));
    B = [revoluteBase zeros(size(revoluteBase,1), 6);zeros(6,size(revoluteBase,2)) eye(6)];
    Total_J = zeros(6 * nData, nQ * nData);
    Total_L = zeros(6 * nData, 6 * (nJoint + 1));
    for i = 1:nData
        [~, J] = forwardKinematics(screws(:,1:end-1), screws(:,end), thetas(i,:));
        Total_J(6 * (i-1) + 1 : 6 * i, nQ * (i-1) + 1 : nQ * i) = J * dq_dtheta;
        Total_L(6 * (i-1) + 1 : 6 * i, :) = get_L(screws, thetas(i,:));
    end
    Total_J_perp = eye(6 * nData) - Total_J / (Total_J'*Total_J) * Total_J';
    [~,S,V] = svd(Total_J_perp * Total_L * B);
    %[~,S,V] = svd(Total_J_perp * Total_L );
    dim_of_null = sum(diag(S) < eig_thld);
    if dim_of_null == 0
        error('failed to find del_eta')
    else
        del_eta = B * V(:, end - dim_of_null + 1 : end);
        %del_eta = V(:, end - dim_of_null + 1 : end);
    end
end