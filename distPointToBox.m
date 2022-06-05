function [dist, unit] = distPointToBox(p, box)
%
% INPUTS
%   p   - Nx2 vector of [x,y] coordinates of the point(s)
%   box - 1x4 vector of minimum and maximum points for an axis-aligned 
%         boundary box 
% 
% OUTPUTS
%   dist - Nx1 vector of distances between the point(s) and the box
%
%           dist > 0 -> point is outside the box
%           dist = 0 -> point is on/inside the box
%
%   unit - Nx3 array where each row is the corresponding unit vector
%           to the closest spot on the box.
%
%           norm(unit) = 1 -> point is outside the box
%           norm(unit) = 0 -> point is on/inside the box
%
% AUTHOR
%   Gedaliah Knizhnik (knizhnik@seas.upenn.edu) - 10/23/19
%   Method from MultiRRomero @ https://stackoverflow.com/questions/5254838/calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
%   Modified by Zachary Fisher & Michael Woc

%%

% Get box info
boxMin    = box(1:2);
boxMax    = box(3:4);
boxCenter = (boxMin + boxMax)/2;

% Get distance info, where we use 0 if the point is between the box
% boundary along this axis (i.e. it will be closest to 1-dimension down)
dx = max([boxMin(1) - p(:,1),zeros(size(p,1),1), p(:,1) - boxMax(1)],[],2);
dy = max([boxMin(2) - p(:,2),zeros(size(p,1),1), p(:,2) - boxMax(2)],[],2);

% Calculate the distances
dist = vecnorm([dx,dy],2,2);

% Figure out the signs:
sgns = sign(boxCenter - p);

% Calculate the unit vectors and replace any Inf or NaN values (from 0
% distances) with 0.
unit = [dx,dy]./dist.*sgns;
unit(isinf(unit)) = 0;
unit(isnan(unit)) = 0;

end