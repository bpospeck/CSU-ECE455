% Forward kinematics performed
% Args: n, l, theta
% Returns: actual [x,y] positions
function [x,y] = forKinematics(n,l,theta)
    x = 0.0;
    y = 0.0;
    angleSum = 0.0;
    for i = 1:n
        angleSum = angleSum + theta(i);
        x = x + l(i)*cos(angleSum);
        y = y + l(i)*sin(angleSum);
    end
end