function updatedScrew = updateScrew(initialScrew, delta)
nJoint = size(initialScrew,2);
if((size(initialScrew,1)~=6) || (length(delta)~=(6*nJoint)))
    error('[ERROR] Wrong input for updateScrew()')
end
updatedScrew = initialScrew;
weight_w    = 1;
weight      = [ones(3,1) ; 1/weight_w * ones(3,1)];
for i=1:nJoint
    updatedScrew(:,i) = largeAdjoint(expm(ToMatrix(weight .* delta(6*(i-1)+1:6*i)))) * initialScrew(:,i);
end
end