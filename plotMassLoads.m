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