function [EEpose, SpaceJacobian] = forwardKinematics(A_screw,  M_screw, jointAngle)
nJoint = length(jointAngle);

if any(size(A_screw)~=[6 nJoint]) || length(M_screw)~=6
   error(['forwardKinematics' num2str(size(A_screw))]) 
end

SpaceJacobian = zeros(6,nJoint);
expSeries   = eye(4);
for i=1:nJoint
    SpaceJacobian(:,i) = largeAdjoint(expSeries) * A_screw(:,i);
    expSeries = expSeries * expm(ToMatrix(A_screw(:,i) * jointAngle(i)));
end
EEpose  = expSeries * expm(ToMatrix(M_screw));

end