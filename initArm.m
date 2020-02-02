% Initializes all characteristics of arm system: link and joint properties,
% rigid transform matrices, and rotation matricies
%
% prm - cell array of all input arm parameters
%       prm(i,:) = {link name, mass, a_ij, alpha_ij,
%             joint name, mass, S_i, theta_i};
% k - struct of gravity parameters
%
% links - structure array with properties of every link
% joints - structure array with properties of every joint
% T - structure array with all transformation matrices [see getTransform]
% R - structure array with all rotation matrices [see getTransform]

function [links, joints, T, R] = initArm(prm,k)
[num,~] = size(prm);    % number of joints on arm
for n = 1:num   % initialize independent link/joint properties
    links(n) = linkBuilder(prm{n,1},n,prm{n,2},prm{n,3},prm{n,4},k);
    joints(n) = jointBuilder(prm{n,5},n,prm{n,6},prm{n,7},prm{n,8},k);
end
[T,R] = getTransform(links,joints); % get transform & rotation matrices
for n = 1:num   % get global coordinates of points of interest
    O = T(n).Fi*[0;0;0;1];  O = O(1:3);
    J = T(n).Fi*[-joints(n).localVec;1];    J = J(1:3);
    L = T(n).Fi*[links(n).localVec;1];      L = L(1:3);
    joints(n).endpoints = [J O];    % TODO: uhhhhh, why it be that
    links(n).endpoints = [O L];
    
    J = T(n).Fi*[-joints(n).cmLocal;1];    J = J(1:3);
    L = T(n).Fi*[links(n).cmLocal;1];      L = L(1:3);
    joints(n).cmGlobal = J;
    links(n).cmGlobal = L;
end
end