function [c, ceq, gc, gceq] = EEpose_contraint(Screw_and_jointAngle, nJoint, T)
ceq = 0;
gceq = zeros(size(Screw_and_jointAngle));
Screw = reshape(Screw_and_jointAngle(1:6 * (nJoint + 1)), 6, []);
jointAngle = reshape(Screw_and_jointAngle( (6 * (nJoint + 1)) + 1 : end ), [], nJoint);
nData = length(jointAngle(:,1));
if length(Screw(:,1)) ~= 6 || length(jointAngle(1,:)) + 1 == length(Screw(1,:))
    error('getManipulability : Input dimension error')
end
for i = 1:nData
    [EE, ~] = forwardKinematics(Screw(:,1:end-1), Screw(:,end), jointAngle(i,:));
    ceq = ceq + norm(EE - reshape(T(i,:,:),4,4));
end
c = [];
gc = [];
end