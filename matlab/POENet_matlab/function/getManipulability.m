function [m, grad_theta, grad_eta] = getManipulability(jointAngle, Screw, nJoint, dq_dtheta)
if length(Screw(:,1)) ~= 6 || length(jointAngle(1,:)) + 1 == length(Screw(1,:)) || length(jointAngle(1,:)) == length(dq_dtheta(:,1))
    error('getManipulability : Input dimension error')
end
if length(jointAngle(1,:)) > 6 
    error('It is redundant mechanism. This function only works for deficient mechanism');
end
nData = length(jointAngle(:,1));
m = 0;
grad_theta = zeros(size(jointAngle));
grad_eta = zeros(size(Screw));
for i = 1:nData
    [~, SpaceJacobian] = forwardKinematics(Screw(:,1:end-1), Screw(:,end), jointAngle(i,:));
    J = SpaceJacobian * dq_dtheta;
    m = m + log(det(J'*J));
    dJ1 = getDifferentialJacobian_wrt_theta(SpaceJacobian);
    dJ2 = getDifferentialJacobian_wrt_screw(SpaceJacobian, Screw, jointAngle(i,:));
    for j = 1:nJoint
        grad_theta(i,j) = sum(2*pinv(J)' .* (dJ1(:,:,j) * dq_dtheta), 'all');
    end
    for j = 1 : nJoint
        grad_eta(:,j) = grad_eta(:,j) - sum( 2*pinv(J)' .* (squeeze(dJ2(:,k,j,:)) * dq_dtheta), 'all') * jointAngle(i,j);
    end 
end
m = m / nData;
grad_theta = grad_theta / nData;
grad_eta = grad_eta / nData;
end