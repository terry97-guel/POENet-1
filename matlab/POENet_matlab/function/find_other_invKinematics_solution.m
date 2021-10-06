function theta = find_other_invKinematics_solution(T, A, init_theta, dir, max_iter)
    count = 0;
    nJoint = length(init_theta);
    if any(size(T) ~= [4,4]) || any(size(A) ~= [6, nJoint+1]) || any(size(init_theta)~=[nJoint,1]) || any(size(dir) ~= [nJoint,1]) ||any(size(max_iter) ~= [1,1])
        error('Wrong input for find_other_invKinematics_solution()')
    end
    temp_theta = init_theta' + dir';
    T_se3 = transpose(ToVector(logm(T)));
    [temp_theta, ~, ~, EFerror] = solveInverseKinematics(T_se3, A(:,1:end-1), A(:,end), temp_theta);
    while(norm(init_theta' - temp_theta) < 0.001 || norm(EFerror) > 0.001)
        if count > max_iter
            error('failed to find other inverse kinematics solution');
        end
        temp_theta = init_theta' + dir' * (count+2);
        [temp_theta, ~, ~, EFerror] = solveInverseKinematics(T_se3, A(1:end-1,:), A(end,:), temp_theta);
        count = count + 1;
    end
    theta = temp_theta;
end
