% Uses torque functions for each joint to calculate loads for a full path
% 
% jointArray - structure array of joint parameters
% q0 - array of angular positions of each timestep
% q1 - array of angular velocities of each timestep
% q2 - array of angular accelerations of each timestep
%
% torques - array of load for all joints and times
%           [jointID, timestep, load component]

function torques = getJointLoads(jointArray,q0,q1,q2)
[num,tt] = size(q0);
torques = zeros(num,tt,3);
for n = 1:num
    f = jointArray(n).torques;
    for t = 1:tt
        torques(n,t,:) = f(q0(:,t),q1(:,t),q2(:,t));
    end
end
end