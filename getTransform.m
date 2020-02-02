% Calculates all rigid transforms using given parameters of joint and link
% lengths and angles.
% see: Crane & Duffy, Kinematic Analysis of Robot Manipulators, Chap. 3.6
% 
% linkArray - structure array of link parameters
% jointArray - structure array of joint parameters
%
% T - struct of rigid transforms:   ij - down to earlier axes
%                                   ji - up to next axes
%                                   Fi - down to fixed (global) axes
%                                   Ej - up to tool axes
% R - struct of rotation matrices:  same fields as T
% T = [      x;     Rotation matrix is upper left 3x3 of an individual
%        R   y;     4x4 rigid transform.
%            z;
%      0,0,0,1;]

function [T,R] = getTransform(linkArray, jointArray)
T = struct('ij',cell(1,6),'ji',[],'Fi',[],'Ej',[]); % memory allocation
R = struct('ij',cell(1,6),'ji',[],'Fi',[],'Ej',[]); % memory allocation
c_j = cosd(jointArray(1).theta);
s_j = sind(jointArray(1).theta);

T(1).ij = [ c_j,    -s_j,   0,  0;...   % No a_F1 vector so the first
            s_j,    c_j,    0,  0;...   % transform is a special case
            0,      0,      1,  0;...
            0,      0,      0,  1 ];

T(1).ji = [ c_j,    s_j,    0,  0;...
            -s_j,   c_j,    0,  0;...
            0,      0,      1,  0;...
            0,      0,      0,  1 ];
        
R(1).ij = T(1).ij(1:3,1:3);  R(1).ji = T(1).ji(1:3,1:3);

for n = 2:numel(jointArray) % creating all 'ij' and 'ji' matrices
    S_j = jointArray(n).S;
    c_j = cosd(jointArray(n).theta);
    s_j = sind(jointArray(n).theta);
    a_ij = linkArray(n-1).a;
    c_ij = cosd(linkArray(n-1).alpha);
    s_ij = sind(linkArray(n-1).alpha);

    T(n).ij = [ c_j,        -s_j,       0,      a_ij;...
                s_j*c_ij,   c_j*c_ij,   -s_ij,  -s_ij*S_j;...
                s_j*s_ij,   c_j*s_ij,   c_ij,   c_ij*S_j;...
                0,          0,          0,      1 ];

    T(n).ji = [ c_j,    s_j*c_ij,   s_j*s_ij,   -c_ij*a_ij;...
                -s_j,   c_j*c_ij,   c_j*s_ij,   s_j*a_ij;...
                0,      -s_ij,      c_ij,       -S_j;...
                0,      0,          0,          1 ];
        
    R(n).ij = T(n).ij(1:3,1:3); R(n).ji = T(n).ji(1:3,1:3);  
end

for n = 1:numel(jointArray) % calculating cumulative transform matrices
    T_Fi = eye(4);  R_Fi = eye(3);
    for m = 1:n     % T_Fi = T_F1*...*T_(m-1)m
        T_Fi = T_Fi*T(m).ij;
        R_Fi = R_Fi*R(m).ij;
    end
    T(n).Fi = T_Fi; R(n).Fi = R_Fi;
    
    T_Ej = eye(4);  R_Ej = eye(3);
    for m = n:6     % T_Ej = T_F1*...*T_(m-1)m
        T_Ej = T_Ej*T(m).ji;
        R_Ej = R_Ej*R(m).ji;
    end
    T(n).Ej = T_Ej; R(n).Ej = R_Ej;
end
end
