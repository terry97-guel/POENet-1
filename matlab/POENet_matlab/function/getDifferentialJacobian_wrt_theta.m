function dJ = getDifferentialJacobian_wrt_theta(J)
if length(J(:,1)) ~= 6
    error('getDifferentialJacobian_wrt_theta : input dimension error')
end
nJoint = length(J(1,:));
dJ = zeros([size(J),nJoint]);
for i = 1:nJoint
    for l = 1:nJoint
        if i >= l 
            dJ(:,l,i) = smallAdjoint(J(:,i)) * J(:,l);
        end
    end
end
end