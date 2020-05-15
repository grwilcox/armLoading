% Plots skeleton structure of arm. Uses endpoints of each segment
% calculated during initialization.
% 
% torques - array of load for all joints and times
%           [jointID, timestep, load component]
% tt - array of timesteps
% jointArray - structure array of joint parameters
% n - ID specify which joint to plot loads of

function plotLoads(loads,tt,jointArray,n)
figure(2)
hold on
xlabel('time [s]')
ylabel('Torque [N*m]')
plot(tt,loads(n,:,1))
plot(tt,loads(n,:,2))
plot(tt,loads(n,:,3))
plot(tt,sum(loads(n,:,:),3))
legend('gravity','velocity','acceleration','total')
title(['Loads on ' jointArray(n).name])