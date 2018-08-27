% Bradley Pospeck
% ECE455 program project 1
% read in 'robot.key' and 'object.key'

[k, t] = readtk('robot.key');
M = dlmread('robot.key','',1,0);    % Read in position and velocity
rPos = M(1:2:end,:);                % Extract positions into rPos
rVel = M(2:2:end,:);                % Extract velocities into rVel
interpolateJoints(k, t, rPos, rVel);

[k, t] = readtk('object.key');
M = dlmread('object.key','',1,0);
M;              % Extract matrix data
R = M(:,1:3);   % Store all R's here
P = M(:,4);     % Store all P's here
% Setup P's in xyz form per row
P = reshape(P,[1,k*3]);
Pn = [];
for i = 1:k
    Pn = [Pn; P(:,3*i-2:3*i) ];
end
Pnew = interpolateObj(k,t,Pn);
Q = [];
for i = 1:k
    Q = [Q; R_to_Q( R(3*i-2:3*i,:) )];
end
Qnew = Q_interpolation(Q,k,t);
Rnew = [];
for i = 1:t
    Rnew = [Rnew; Q_to_R(Qnew(i,:))];
end
writeObjTraj(t,Pnew,Rnew);