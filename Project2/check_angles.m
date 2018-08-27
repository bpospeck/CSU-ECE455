% Plot the desired end effector position as 'o'
% Show actual end effector position as 'x'

function check_angles
    load trajectory
    trajectory=trajectory(2:end,:); % Remove first line of values
    load angles
    load arm
    [m,n]=size(angles);
    dtheta=diff(angles);
    link_length=arm(2:n+1,1);
    
    x_actual=zeros(m,2);
    error=zeros(m,1);
    mag_dtheta=zeros(m-1);
    for i=1:m-1
        mag_dtheta(i)=norm(dtheta(i,:));
    end

    for i=1:m
        x_actual(i,:)=[0,0];
        sum=cumsum(angles(i,:))';
        x_actual(i,1)=x_actual(i,1)+link_length'*cos(sum);
        x_actual(i,2)=x_actual(i,2)+link_length'*sin(sum);
        
        x_error=trajectory(i,:)-x_actual(i,:);
        error(i)=x_error*x_error';
    end
    
    figure(3)
        th=linspace(-pi/2,pi/2,100);
        R=ones(1,n)*link_length;
        plot(trajectory(:,1),trajectory(:,2),'ob')
        hold on
        plot(x_actual(:,1),x_actual(:,2),'+r')
        plot(R*cos(th),R*sin(th),'g')
        axis([min(trajectory(:,1))-0.5,max(trajectory(:,1))+0.5...
            min(trajectory(:,2))-0.5,max(trajectory(:,2))+0.5])
        title('Desired and Actual Position Trajectories')
        xlabel('x_1')
        ylabel('x_2')
        %legend('x_{desired}','x_{actual}','Workspace Boundary')
end