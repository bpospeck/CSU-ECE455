% Take in position and velocity information at k key points
% interpolate t joint outputs and write those outputs to robot.ang
function interpolateJoints(k,t,pos,vel)
    interval = (k-1)/(t-1);     % interval size for u during interp.
    u = 0.0;                        
    Q = [];                 % Empty matrix for joint angle storage
    for i = 1:k-1
        p_i  = pos(i,:);
        p_i1 = pos(i+1,:);
        d_i  = vel(i,:);
        d_i1 = vel(i+1,:);
        while u < 1.0
            Q = [Q; getHermiteP(u, p_i, p_i1, d_i, d_i1)];
            u = u + interval;
        end
        u = u-1.0;
    end     
    dlmwrite('robot/robot.ang',t);
    dlmwrite('robot/robot.ang',Q,'-append','delimiter',' ');
end