% Perform Bezier interpolation between quaternions

function stacked_q=Q_interpolation(key_q,key_frames,total_frames)
    du=(key_frames-1)/(total_frames-1);
    a=zeros(key_frames-2,4);
    b=zeros(key_frames-2,4);
    a(1,:)=key_q(1,:);
    b(5,:)=key_q(5,:);
    for i=2:key_frames-1
        a(i,:)=Bisect(Double(key_q(i-1,:),key_q(i,:)),key_q(i+1,:));
		a(i,:)=slerp(key_q(i,:),a(i,:),1.0/3.0);
        b(i,:)=Double(a(i,:),key_q(i,:));        
    end

    qu=zeros(total_frames,4);
    j=1;
    for i=1:key_frames-1
        for u=0:du:1
            q_new=slerp(a(i,:),b(i+1,:),u);
            q_new=slerp(slerp(slerp(key_q(i,:),a(i,:),u),q_new,u),...
                slerp(q_new,slerp(b(i+1,:),key_q(i+1,:),u),u),u);
            if (j>1)
                q1=qu(j-1,:)*q_new';
                if (q1>0)
                    qu(j,:)=q_new;
                else
                    qu(j,:)=[-1 -1 -1 1].*q_new;
                end
            end
            j=j+1;
        end
    end
    stacked_q=qu;
    [v,w]=size(stacked_q)
end