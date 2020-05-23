% Plots skeleton structure of arm. Uses endpoints of each segment
% calculated during initialization.
% 
% torques - array of load for all joints and times
%           [jointID, timestep, load component]
% tt - array of timesteps
% jointArray - structure array of joint parameters
% n - ID specify which joint to plot loads of 
%       OR 'total' for plot of total torques for all joints

function plotLoads(loads,tt,jointArray,n)
figure(nextFig)
hold on
xlabel('time [s]')
ylabel('Torque [N*m]')
if isnumeric(n)
    plot(tt,loads(n,:,1))
    plot(tt,loads(n,:,2))
    plot(tt,loads(n,:,3))
    plot(tt,sum(loads(n,:,:),3))
    legend('gravity','velocity','acceleration','total')
    title(['Loads on ' jointArray(n).name])
elseif n == 'total'
    for i = 1:size(loads,1)
        plot(tt,sum(loads(i,:,:),3))
    end
    legend(jointArray(1).name,jointArray(2).name,jointArray(3).name,...
        jointArray(4).name,jointArray(5).name,jointArray(6).name)
    title('Total loads on each joint')
else
    error('no');
end
end
