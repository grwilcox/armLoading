% Calculates inertia tensor around pivot given center of mass inertia
% tensor and relative coordinate of pivot. Then reorders matrix to
% correspond to relabeled coordinate orientation.
% 
% Icom - center of mass inertia tensor
% disp - displacement vector to new pivot (column vector)
% m - total mass of object
% axes - relabeling order of axes, eg. [3,1,2]: (x,y,z) -> (z,x,y)
%
% Ipiv - inertia tensor around new pivot

function Ipiv = parallelAxis(Icom,disp,m,axes)
A = Icom + m*(disp.'*disp)*eye(3) - m*(disp*disp.');
r = axes(1);    s = axes(2);    t = axes(3);
B = [A(:,r),A(:,s),A(:,t)];
Ipiv = [B(r,:);B(s,:);B(t,:)];