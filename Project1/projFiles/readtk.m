% Read file headers to extract number of key frames and total frames
% Given a file name
function [k, t]=readtk(fileName)
    inFile = fopen(fileName);
    k = fscanf(inFile, '%d', 1);
    t = fscanf(inFile, '%d', 1);
    fclose(inFile);
end