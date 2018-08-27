clear
load path
load angles
[m,n]=size(angles)
for i = 2:m, 
norm(i) = 0;
xa(i) = 0;
ya(i) = 0;
sum = 0;
for j = 1:n, 
diff = angles(i,j) - angles(i-1,j);
%diff = (angles(i,j) - angles(i-1,j))*3.14159/180.0;
norm(i) = norm(i) + diff*diff;
sum = sum + angles(i,j);
%sum = sum + angles(i,j)*3.14159/180.0;
xa(i) = xa(i) + 10*cos(sum);
ya(i) = ya(i) + 10*sin(sum);
end;
xd(i) = path(i,1);
yd(i) = path(i,2);
%
% Depends on whether you assume manipulator
% starts on first point in trajectory
%
xe = path(i,1)-xa(i);
ye = path(i,2)-ya(i);
%xe = path(i-1,1)-xa(i);
%ye = path(i-1,2)-ya(i);
error(i) = xe*xe + ye*ye;
end;
plot(xd,yd,'o',xa,ya,'+')
axis([37,40,0,20])
%axis([-20,40,-20,50])
pause;
subplot(2,1,1)
plot(norm);
axis([0,50,0,.05])
subplot(2,1,2)
plot(error);
axis([0,50,0,1])
