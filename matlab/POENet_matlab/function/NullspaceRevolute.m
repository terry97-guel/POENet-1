function nullbase = NullspaceRevolute(mat_screw)
nJoint = size(mat_screw,2);
if(size(mat_screw,1)~=6)
    error('ERROR: Wrong input for NullspaceRevolute()')
end
nullbase = zeros(6*nJoint,4*nJoint);
for i=1:nJoint
    w = mat_screw(1:3,i);
    v = mat_screw(4:6,i);
    nullbase(6*(i-1)+1:6*i,4*(i-1)+1:4*i) = null([w zeros(size(w));v w]');
end
end