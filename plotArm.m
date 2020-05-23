% Plots skeleton structure of arm. Uses endpoints of each segment
% calculated during initialization.
% 
% linkArray - structure array of link parameters
% jointArray - structure array of joint parameters
% q - array of angular positions of each timestep

function plotArm(linkArray,jointArray,q)
[num,tt] = size(q);
p1 = zeros(3,num,tt); p2 = p1; p3 = p1;
for t = 1:tt
for n = 1:num
    p1(:,n,t) = jointArray(n).endp1(q(:,t));
    p2(:,n,t) = jointArray(n).endp2(q(:,t));
    p3(:,n,t) = linkArray(n).endp3(q(:,t));
end
end
f1 = figure (nextFig);
xlabel('x [in]'); ylabel('y [in]'); zlabel('z [in]');
xlims = [min([p1(1,:,:) p2(1,:,:) p3(1,:,:)],[],'all'),...
    max([p1(1,:,:) p2(1,:,:) p3(1,:,:)],[],'all')];
ylims = [min([p1(2,:,:) p2(2,:,:) p3(2,:,:)],[],'all'),...
    max([p1(2,:,:) p2(2,:,:) p3(2,:,:)],[],'all')];
zlims = [min([p1(3,:,:) p2(3,:,:) p3(3,:,:)],[],'all'),...
    max([p1(3,:,:) p2(3,:,:) p3(3,:,:)],[],'all')];
axis equal;
view([52.5 30]);
for t = 1:tt
for n = 1:num
    plot3([p1(1,n,t),p2(1,n,t)],[p1(2,n,t),p2(2,n,t)],...
        [p1(3,n,t),p2(3,n,t)],'o-b','MarkerIndices',2,...
        'LineWidth',3,'MarkerSize',12);
    if (n==1); hold on; grid on; end
    % after first plot3, or else it forces 2D axes
    plot3([p2(1,n,t),p3(1,n,t)],[p2(2,n,t),p3(2,n,t)],...
        [p2(3,n,t),p3(3,n,t)],'s-m','MarkerIndices',2,...
        'LineWidth',3,'MarkerSize',12);
end
hold off
% if (t<tt); input("next");
pause(.25);
end
end
