function dJ = getDifferentialJacobian_wrt_screw(J, screw, theta)
if length(J(:,1)) ~= 6 || length(screw(:,1)) ~= 6 || length(theta) ~= length(J(1,:))
    error('getDifferentialJacobian_wrt_screw : input dimension error')
end
nJoint = length(theta);
dJ = zeros([6, 6, nJoint, nJoint]);
poe = eye(6);
for i = 1:nJoint
    for l = 1:nJoint
        if i == l
            dJ(:,:,i,l) = LargeAdjoint(poe);
        elseif i < l
            dJ(:,:,i,l) * theta(i) * smallAdjoint(J(:,l)) * LargeAdjoint(poe);
        end
    end    
    poe = poe * expm(ToMatrix(screw(:,i)*theta(i)));
end
end