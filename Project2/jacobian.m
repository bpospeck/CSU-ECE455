% Find the jacobian
% args: n, l, theta
% Returns: Jacobian of size 3xn
function J = jacobian(n, l, theta)
    %a should always be (0,0,1) for xy planar robot
    d = 0.0; % Always 0 for xy planar robot with rotary
    alpha = 0.0; % Always 0 for xy planar robot
    J = zeros(3,n);
    A = [1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];
    for i=n:-1:1 % start with the last link and work backwards
        ll = l(i); %link length is current link
        angle = theta(i);
        A_i = [cos(angle),-cos(alpha)*sin(angle),sin(alpha)*sin(angle),ll*cos(angle);...
            sin(angle),cos(alpha)*cos(angle),-sin(alpha)*cos(angle),ll*sin(angle);...
            0,sin(alpha),cos(alpha),d; 0,0,0,1];
        A = A_i*A; % getting transform from current joint to tool tip
        a = A(1:3,3); % a vector
        p = A(1:3,4); % p vector
        j_i = cross(a,p);
        J(:,i) = j_i;
    end
    J = J(1:2,:); % Don't need the 3rd row for planar robot
    disp(J)
end