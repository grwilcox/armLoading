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