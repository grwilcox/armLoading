% Initializes all characteristics of arm system: link and joint properties,
% rigid transform matrices, and rotation matricies
%
% prm - cell array of all input arm parameters
%       prm(i,:) = {link name, mass, a_ij, alpha_ij,
%             joint name, mass, S_i, theta_i};
%
% links - structure array with properties of every link
% joints - structure array with properties of every joint
% T - structure array with all transformation matrices [see getTransform]
% R - structure array with all rotation matrices [see getTransform]

function [links, joints, T, R] = initArm(prm)
[num,~] = size(prm);    % number of joints on arm
syms q [num 1] real     % symbolic joint angles
syms qd [num 1] real    % symbolic joint velocities
syms qdd [num 1] real   % symbolic joint acceleration

k.g = 9.81;
k.kHat = [0; 0; 1];
k.m_in = .0254; % inches to meters, customary units are stupid.

for n = 1:num   % initialize independent link/joint properties
    links(n) = linkBuilder(prm{n,1},n,prm{n,2},prm{n,3},k);
    joints(n) = jointBuilder(prm{n,4},n,prm{n,5},prm{n,6},prm{n,7},...
        prm{n,8},prm{n,9},q(n),qd(n),qdd(n),k);
end
[T,R] = getTransform(links,joints); % get transform & rotation matrices
[joints] = initJointLoads(joints,T,k);
for n = 1:num   % get global coordinates of points of interest
    O = T(n).Fi*[0;0;0;1];  O = O(1:3);
    J = T(n).Fi*[-joints(n).localVec;1];    J = J(1:3);
    L = T(n).Fi*[links(n).localVec;1];      L = L(1:3);
    joints(n).endp1 = matlabFunction(J,'Vars',{q});
    joints(n).endp2 = matlabFunction(O,'Vars',{q});
    links(n).endp3 = matlabFunction(L,'Vars',{q});
    % TODO: uhhhhh, why it be like that
    
    J = T(n).Fi*[-joints(n).cmLocal;1];    J = J(1:3);
    joints(n).cmGlobal = matlabFunction(J,'Vars',{q});
    
%     joints(n).torques = matlabFunction(joints(n).torques,'Vars',{q,qd,qdd});
    joints(n).gravT = matlabFunction(joints(n).gravT,'Vars',{q,qd,qdd});
    joints(n).veloT = matlabFunction(joints(n).veloT,'Vars',{q,qd,qdd});
    joints(n).accelT = matlabFunction(joints(n).accelT,'Vars',{q,qd,qdd});
end
end
