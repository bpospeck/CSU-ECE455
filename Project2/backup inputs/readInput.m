% Read 'arm' and 'trajectory' file inputs
% takes fileName as an argument
% returns 4 vars based on input file
function [nm,lambda,lx,ytheta]=readInput(fileName)
    input = fopen(fileName);
    % nm could have input of either n or m
    nm = fscanf(input, '%d', 1); 
    lambda = fscanf(input, '%f', 1);
    fclose(input);
    
    M = dlmread(fileName,'',1,0); 
    % lx may have input of link length or x desired
    lx = M(:,1);
    % ytheta may have input of joing angle or y desired
    ytheta = M(:,2);
end