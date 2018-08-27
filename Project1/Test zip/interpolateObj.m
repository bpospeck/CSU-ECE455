function Pnew=interpolateObj(k,t,pos)
    interval = (k-1)/(t-1);     % interval size for u during interp.
    u = 0.0;                        
    temp = [];                 % Empty matrix for joint angle storage
    for i = 1:k-1
        if i == 1
            p_i0 = pos(i,:);
        else
            p_i0  = pos(i-1,:);
        end
        p_i1 = pos(i,:);
        if i+2 == k
            p_i2 = pos(i+1,:);
            p_i3 = p_i2 + (p_i2 - p_i1);
        elseif i+1 == k
            p_i2 = p_i1 + (p_i1 - p_i0);
            p_i3 = p_i2 + (p_i2 - p_i1);
        else
            p_i2 = pos(i+1,:);
            p_i3 = pos(i+2,:); 
        end
        while u < 1.0
            temp = [temp; getCatmullRomP(u, p_i0, p_i1, p_i2, p_i3)];
            u = u + interval;
        end
        u = u-1.0;
    end
    Pnew = temp;
end