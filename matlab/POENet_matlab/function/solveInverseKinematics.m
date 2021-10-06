function [angle, poserror, orierror, EFerror] = solveInverseKinematics(EFpose, A_screw, M_screw, angle_init)
nJoint      = size(A_screw,2);
nData       = size(EFpose,1);
if size(EFpose,2) ~= 6 || any(size(A_screw)~=[6,nJoint]) || length(M_screw)~=6 || any(size(angle_init)~=[nData, nJoint])
    error('Wrong input for solveIK()')
end
bodyJacobian = zeros(6, nJoint);
M = expm(ToMatrix(M_screw));
stepsize = 0.01;
finished = zeros(nData,1);
poserror = zeros(nData,1);
orierror = zeros(nData,1);
EFerror = zeros(nData,6);
angle = angle_init;
while true
    % EF error & derivative
    for t=1:nData
        if finished(t)
            continue
        else
            expSeries = eye(4);
            for i=nJoint:-1:1
                A_i = A_screw(:,i);
                bodyJacobian(:,i) = largeAdjoint(expSeries) * A_i;
                expSeries = expSeries * expm(ToMatrix(-A_i * angle(t,i)));
            end
            nominal = inverseSE3(expSeries) * M;
            actual = expm(ToMatrix(EFpose(t,:)));
            SE3diff = nominal \ actual;
            EFerror(t,:) = ToVector(logm(SE3diff))';
            poserror(t) = norm(SE3diff(1:3,4));
            orierror(t) = norm(ToVector(logm(SE3diff(1:3,1:3))));
            if poserror(t) < 0.01
                finished(t) = 1;
                continue
            end
            angle(t,:) = angle(t,:) + (stepsize*pinv(bodyJacobian)*EFerror(t,:)')';
        end
    end
    if all(finished)
        break
    end
end

end