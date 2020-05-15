% Parses data from IK solver output describing desired path over time.
% 
% filename - .txt file containting IK output data
%             (yes you need to type the extension)
%
% q0 - array of angular positions of each timestep
% q1 - array of angular velocities of each timestep
% q2 - array of angular accelerations of each timestep
% tt - array of timesteps

function [q0, q1, q2, tt] = parseIKoutput(filename)
fileID = fopen(filename);
C = textscan(fileID,'%s','Delimiter','\n');
C = C{1};
tSteps = size(C,1)/8-1;
tt = zeros(1,tSteps);
q0 = zeros(6,tSteps);
q1 = q0;    q2 = q0;

for n = 1:tSteps
    q0(:,n) = cell2mat(textscan(C{2+n*8},...
        'positions: [%f, %f, %f, %f, %f, %f]'));
    q1(:,n) = cell2mat(textscan(C{3+n*8},...
        'velocities: [%f, %f, %f, %f, %f, %f]'));
    q2(:,n) = cell2mat(textscan(C{4+n*8},...
        'accelerations: [%f, %f, %f, %f, %f, %f]'));
    sec = cell2mat(textscan(C{7+n*8},'secs: %f'));
    nsec = cell2mat(textscan(C{8+n*8},'nsecs: %f'));
    tt(n) = sec + nsec/1e9;
end
