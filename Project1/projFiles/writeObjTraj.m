% Combine R and P's after interpolation and write to output
function writeObjTraj(t,P,R)
    dlmwrite('object/object.traj',t);
    for i = 1:t
       for j = 1:3
           out = [R((3*(i-1))+j,:),P(i,j)];
           dlmwrite('object/object.traj',out,'-append','delimiter',' '); 
       end
    end
    %dlmwrite('robot/robot.ang',Q,'-append','delimiter',' ');
end