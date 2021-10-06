function L = get_L(screws, theta)
    nJoints = length(theta);
    poe = eye(4);
    L = zeros(6,6*(nJoints+1));
    for i = 1:nJoints
        L(:,6*(i-1)+1:6*i) = largeAdjoint(poe);
        poe = poe * expm(ToMatrix(screws(:,i))*theta(i));
        L(:,6*(i-1)+1:6*i) = L(:,6*(i-1)+1:6*i) - largeAdjoint(poe);
    end
    L(:,6*(nJoints)+1 : 6*(nJoints+1)) = largeAdjoint(poe);
end