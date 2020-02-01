
clc
clear all

k.g = 9.81;
k.kHat = [0; 0; 1];

% specs = {link: mass, a_ij, alpha_ij, joint: mass, S_i, theta_i}
% sp =   {1, 0, 270,      1, 0, 0;...
%         1, 13.55, 0,    1, 0, 270;...
%         1, 0, 90,       1, -3.564, 90;...
%         1, 0, 270,      1, 15.159, 0;...
%         1, 0, 90,       1, 0, 0;...
%         1, 0, [],       1, 11.876, 0;};
sp =   {0, 0, 270,      1, 0, 30;...
        1, 13.55, 0,    1, 0, 270+18;...
        0, 0, 90,       1, -3.564, 90+80;...
        0, 0, 270,      1, 15.159, 25;...
        0, 0, 90,       1, 0, 45;...
        1, -1, [],      1, 5.876, -35;}; %1, 11.876, -35;};
% syms L1 L2
% sp =   {0, 0, 270,      1, 0, 30;...
%         1, L1, 0,    1, 0, 270+18;...
%         0, 0, 90,       1, -3.564, 90+80;...
%         0, 0, 270,      1, L2, 25;...
%         0, 0, 90,       1, 0, 45;...
%         1, -1, [],      1, 5.876, -35;}; %1, 11.876, -35;};
    

linkArray = [];
jointArray = [];

tempLink = linkBuilder("a12 >> base?",sp{1,1},sp{1,2},sp{1,3},linkArray,jointArray,k);
tempJoint = jointBuilder("S1 >> base rotate",sp{1,4},sp{1,5},sp{1,6},linkArray,jointArray,k);
linkArray = [linkArray, tempLink];  jointArray = [jointArray, tempJoint];

tempLink = linkBuilder("a23 >> upper arm CF",sp{2,1},sp{2,2},sp{2,3},linkArray,jointArray,k);
tempJoint = jointBuilder("S2 >> shoulder",sp{2,4},sp{2,5},sp{2,6},linkArray,jointArray,k);
linkArray = [linkArray, tempLink];  jointArray = [jointArray, tempJoint];

tempLink = linkBuilder("a34 >> elbow complex",sp{3,1},sp{3,2},sp{3,3},linkArray,jointArray,k);
tempJoint = jointBuilder("S3 >> elbow",sp{3,4},sp{3,5},sp{3,6},linkArray,jointArray,k);
linkArray = [linkArray, tempLink];  jointArray = [jointArray, tempJoint];

tempLink = linkBuilder("a45 >> elbow complex 2: kinematic boogaloo",sp{4,1},sp{4,2},sp{4,3},linkArray,jointArray,k);
tempJoint = jointBuilder("S4 >> wrist R1",sp{4,4},sp{4,5},sp{4,6},linkArray,jointArray,k);
linkArray = [linkArray, tempLink];  jointArray = [jointArray, tempJoint];

tempLink = linkBuilder("a56 >> wrist complex",sp{5,1},sp{5,2},sp{5,3},linkArray,jointArray,k);
tempJoint = jointBuilder("S5 >> wrist T",sp{5,4},sp{5,5},sp{5,6},linkArray,jointArray,k);
linkArray = [linkArray, tempLink];  jointArray = [jointArray, tempJoint];

tempLink = linkBuilder("a67 >> EE orient",sp{6,1},sp{6,2},sp{6,3},linkArray,jointArray,k);
tempJoint = jointBuilder("S6 >> wrist R2",sp{6,4},sp{6,5},sp{6,6},linkArray,jointArray,k);
linkArray = [linkArray, tempLink];  jointArray = [jointArray, tempJoint];

T = getTransform(linkArray, jointArray);

% plotArm(linkArray,jointArray,T)
% plotMassLoads(linkArray,jointArray,T,3)

function link = linkBuilder(name, mass, length, angle, linkArray, ~, k)
link.name = name;
link.id = numel(linkArray)+1;
link.mass = mass;
% link.origin = origin;
link.a = length;
link.localVec = [link.a; 0; 0];
link.alpha = angle;
% endPoints = getLinkEnd(link, linkArray);
% link.localEndPoint = endPoints(1,:);
% link.globalEndPoint = endPoints(2,:);
% link.COM = getCOM(link, linkArray);
link.cmVec = link.localVec/2;
link.gravForce = link.mass * k.g * -k.kHat;
% link.trace = [link.origin; link.globalEndPoint];
end

function joint = jointBuilder(name, mass, length, angle, ~, jointArray, k)
joint.name = name;
joint.id = numel(jointArray)+1;
joint.mass = mass;
joint.S = length;
joint.localVec = [0; 0; joint.S];
joint.theta = angle;
joint.cmVec = joint.localVec/2;
joint.gravForce = joint.mass * k.g * -k.kHat;
end

function tStruct = getTransform(linkArray, jointArray)
T = struct('ij',cell(1,6),'ji',[],'Fi',[],'Ej',[]); % memory allocation
c_j = cosd(jointArray(1).theta);
s_j = sind(jointArray(1).theta);

T(1).ij = [c_j, -s_j, 0, 0;...
        s_j, c_j, 0, 0;...
        0, 0, 1, 0;...
        0, 0, 0, 1];

T(1).ji = [c_j, s_j, 0, 0;...
        -s_j, c_j, 0, 0;...
        0, 0, 1, 0;...
        0, 0, 0, 1];
        
for n = 2:numel(jointArray)
    S_j = jointArray(n).S;
    c_j = cosd(jointArray(n).theta);
    s_j = sind(jointArray(n).theta);
    a_ij = linkArray(n-1).a;
    c_ij = cosd(linkArray(n-1).alpha);
    s_ij = sind(linkArray(n-1).alpha);

    T(n).ij = [c_j, -s_j, 0, a_ij;...
            s_j*c_ij, c_j*c_ij, -s_ij, -s_ij*S_j;...
            s_j*s_ij, c_j*s_ij, c_ij, c_ij*S_j;...
            0, 0, 0, 1];

    T(n).ji = [c_j, s_j*c_ij, s_j*s_ij, -c_ij*a_ij;...
            -s_j, c_j*c_ij, c_j*s_ij, s_j*a_ij;...
            0, -s_ij, c_ij, -S_j;...
            0, 0, 0, 1];
end

for n = 1:numel(jointArray)
    T_Fi = eye(4);
    for m = 1:n
        T_Fi = T_Fi*T(m).ij;
    end
    T(n).Fi = T_Fi;
    
    T_Ej = eye(4);
    for m = n:6
        T_Ej = T(m).ji*T_Ej;
    end
    T(n).Ej = T_Ej;
end
tStruct = T;
end

function plotArm(linkArray,jointArray,T)
for n = 1:numel(jointArray)
    O = T(n).Fi*[0;0;0;1];  O = O(1:3);
    J = T(n).Fi*[-jointArray(n).localVec;1];    J = J(1:3);
    L = T(n).Fi*[linkArray(n).localVec;1];      L = L(1:3);
    plot3([J(1) O(1)],[J(2) O(2)],[J(3) O(3)],'o-b','MarkerIndices',2,...
        'LineWidth',1,'MarkerSize',12);
    if (n==1) hold on; end
    plot3([O(1) L(1)],[O(2) L(2)],[O(3) L(3)],'s-m','MarkerIndices',2,...
        'LineWidth',1,'MarkerSize',12);
end
axis equal
xlabel('x [in]'); ylabel('y [in]'); zlabel('z [in]');
hold off
end

function plotMassLoads(linkArray,jointArray,T,scale)
hold on
for n = 1:numel(jointArray)
    cmJ = T(n).Fi*[-jointArray(n).cmVec;1]; cmJ = cmJ(1:3);
    cmL = T(n).Fi*[linkArray(n).cmVec;1];  cmL = cmL(1:3);
    FgJ = cmJ + jointArray(n).gravForce/scale;
    FgL = cmL + linkArray(n).gravForce/scale;
    plot3([cmJ(1) FgJ(1)],[cmJ(2) FgJ(2)],[cmJ(3) FgJ(3)],'d-.r','MarkerIndices',2)
    plot3([cmL(1) FgL(1)],[cmL(2) FgL(2)],[cmL(3) FgL(3)],'d-.r','MarkerIndices',2)
end
axis equal
xlabel('x [in]'); ylabel('y [in]'); zlabel('z [in]');
hold off
end
