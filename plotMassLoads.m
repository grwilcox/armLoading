% Plots vectors representing weight of components over existing arm plot. 
% >>> Must be called AFTER plotArm.
% 
% linkArray - structure array of link parameters
% jointArray - structure array of joint parameters
% scale - factor by which to scale force vector [in/N] 

function plotMassLoads(linkArray,jointArray,scale)
hold on
for n = 1:numel(jointArray)
    cmJ = jointArray(n).cmGlobal;   % coordinates of joint CoM
    cmL = linkArray(n).cmGlobal;    % coordinates of link CoM
    FgJ = cmJ + jointArray(n).gravForce*scale; % translate force vectors
    FgL = cmL + linkArray(n).gravForce*scale;
    plot3([cmJ(1) FgJ(1)],[cmJ(2) FgJ(2)],[cmJ(3) FgJ(3)],'d-.r',...
        'MarkerIndices',2,'LineWidth',.75,'MarkerSize',9)
    plot3([cmL(1) FgL(1)],[cmL(2) FgL(2)],[cmL(3) FgL(3)],'d-.r',...
        'MarkerIndices',2,'LineWidth',.75,'MarkerSize',9)
end
axis equal
xlabel('x [in]'); ylabel('y [in]'); zlabel('z [in]');
hold off
end
