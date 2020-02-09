% Calculates torques on all joints due to gravity, joint velocity, and
% joint accelerations.
% see: Robot Arm Dynamics and Control, NASA document JPL-TM-33-669
% 
% I don't trust this yet.
%
% linkArray - structure array of link parameters
% jointArray - structure array of joint parameters
% T - structure array with all transformation matrices [see getTransform]
% k - struct of gravity parameters

function [linkArray,jointArray] = getJointLoads(linkArray,jointArray,T,~,k)
n = numel(jointArray);
Q = zeros(4); Q(1,2) = -1; Q(2,1) = 1;  % matrix form of partial derivative
G = [0,0,-k.g,1];   % gravity vector
rho = zeros(4,6);   rho(4,:) = 1;
U1 = num2cell(zeros(6,6));  U2 = cell(6,6,6);
J = cell(1,6);  I = J;  k2 = J;
for j = 1:n     % simple initializations (q1,q1,m,rho)
    q1(j) = jointArray(j).velocity; % joint angular velocities
    q2(j) = jointArray(j).acceleration; % joint angular accelerations
    mass(j) = linkArray(j).mass + jointArray(j).mass;   % body j mass
    rho(1:3,j) = linkArray(j).mass*linkArray(j).cmLocal...
        + jointArray(j).mass*jointArray(j).cmLocal; % body j COM
end
for j = 1:n     % init all single derivative U_ji
    for i = 1:j
        U = eye(4); % working matrix
        for m = 1:j
            if m == i
                U = U*Q*T(m).ij;
            else
                U = U*T(m).ij;
            end
        end
        U1{j,i} = U;
    end
end
for j = 1:n     % init all double derivative U_jkp
    for k = 1:j
    for p = 1:j
        U = eye(4); % working matrix
        for m = 1:j
            if (m == k) && (m == p)
                U = U*Q*Q*T(m).ij;
            elseif (m == k) || (m == p)
                U = U*Q*T(m).ij;
            else
                U = U*T(m).ij;
            end
        end
        U2{j,k,p} = U;
    end
    end
end
for j = 1:n     % I and J (pseudo)inertia tensors
    I{j} = diag(linkArray(j).mass*linkArray(j).cmLocal.^2 ...
        + jointArray(j).mass*jointArray(j).cmLocal.^2); % TEMP
    % TODO: make I{j} an input (and find them in Inventor)
    kk = I{j}/mass(j); k2{j} = kk;
    J{j} = mass(j)*[kk+trace(kk)*eye(3)/2-2*eye(3).*kk,...
        rho(1:3,j);rho(:,j).'];
end
for i = 1:n     % F_i torques
    gravT = 0;  veloT = 0;  accelT = 0;
    for j = 1:n
        for k = 1:j
            accelT = accelT + trace(U1{j,k}*J{j}*U1{j,i}.')*q2(k);
            for p = 1:j
            veloT = veloT + trace(U2{j,k,p}*J{j}*U1{j,i}.')*q1(k)*q1(p);
            end
        end
        gravT = gravT - mass(j)*G*U1{j,i}*rho(:,j);
    end
    jointArray(i).torques = [gravT, veloT, accelT];
    jointArray(i).torqueTot = sum([gravT, veloT, accelT]);
end
end