% Plots skeleton structure of arm. Uses endpoints of each segment
% calculated during initialization.
% 
% linkArray - structure array of link parameters
% jointArray - structure array of joint parameters

function plotArm(linkArray,jointArray)
for n = 1:numel(jointArray)
    plot3(jointArray(n).endpoints(1,:),jointArray(n).endpoints(2,:),...
        jointArray(n).endpoints(3,:),'o-b','MarkerIndices',2,...
        'LineWidth',1,'MarkerSize',12);
    if (n==1) hold on; end  % after first plot3, or else it forces 2D axes
    plot3(linkArray(n).endpoints(1,:),linkArray(n).endpoints(2,:),...
        linkArray(n).endpoints(3,:),'s-m','MarkerIndices',2,...
        'LineWidth',1,'MarkerSize',12);
end
axis equal
xlabel('x [in]'); ylabel('y [in]'); zlabel('z [in]');
hold off
end
