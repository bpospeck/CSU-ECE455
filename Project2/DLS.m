% Perform Damped Least Squares
% Args: n,m,l,theta,xDesired,yDesired
% Returns: Joint angles
function thetas= DLS(n,m,lambda,l,theta,xDesired,yDesired)
    %DLS loop
    thetas = [];
    for i = 1:m
       largeError = true;
       iter = 0;
       bound = 0.0001; %error bound
       while largeError
           iter = iter + 1;
           [xActual,yActual] = forKinematics(n,l,theta);
           xErr = xDesired(i) - xActual;
           yErr = yDesired(i) - yActual;
           deltaX = [xErr;yErr];
           J = jacobian(n, l, theta); 
           JT = transpose(J);
           I = eye(2); % create 2x2 identity
           deltaTheta = JT*(J*JT+lambda^2*I)^(-1)*deltaX;
           deltaTheta = transpose(deltaTheta);%make it a row vector like theta
           theta = theta + deltaTheta;
           err = norm(deltaTheta);
           if err < bound
              largeError=false; 
           end
           if iter > 20
               iter = 0;
               bound = bound * 2; %Increase error bound
           end
       end
       thetas = [thetas ; theta];
    end
    %disp(thetas)
end