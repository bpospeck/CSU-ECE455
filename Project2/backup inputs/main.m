% Bradley Pospeck
% ECE455 program project 2: Top level script
[n,lambda,l,theta] = readInput('arm');
theta = transpose(theta); % Needs to be columns
[m,lambda,xDesired,yDesired] = readInput('trajectory');
m = m+1; % One extra position exists for initial
thetas=DLS(n, m, lambda, l, theta, xDesired, yDesired);
dlmwrite('angles',thetas, ' ');