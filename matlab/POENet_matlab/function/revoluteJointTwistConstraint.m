function [c,ceq,gradc,gradceq] = revoluteJointTwistConstraint(del_eta, x_vec)
del_eta_joint = del_eta(1:end-6,:);
nJoint = round(length(del_eta_joint)/6);
nBasis = size(del_eta,2);
if any(size(del_eta_joint)~=[nJoint*6, nBasis]) || any(size(x_vec)~=[nBasis * nBasis,1])
    error(['size(del_eta_joint) = ' num2str(size(del_eta_joint)) ', size(x_vec) = ' num2str(size(x_vec))])
end

c = [];
gradc = [];
ceq = zeros(nBasis*nJoint,1);
gradceq = zeros(length(x_vec), length(ceq));

x_mat = reshape(x_vec,nBasis,nBasis);
screws = del_eta_joint * x_mat;
for i=1:nBasis
    for j=1:nJoint
        % ceq
        screw = screws(6*(j-1)+(1:6),i);
        w = screw(1:3);
        v = screw(4:6);
        ceq(nJoint*(i-1)+j) = w'*v;
        % gradceq
        dscrewdx = del_eta_joint(6*(j-1)+(1:6),:);
        dvdx = dscrewdx(1:3,:);
        dwdx = dscrewdx(4:6,:);
        trans_gradceq = w'*dvdx + v'*dwdx;
        gradceq(nBasis*(i-1)+(1:nBasis), nJoint*(i-1)+j) = trans_gradceq';
    end
end


end


