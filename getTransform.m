function tStruct = getTransform(linkArray, jointArray)
T = struct('ij',cell(1,6),'ji',[],'Fi',[],'Ej',[]); % memory allocation
c_j = cosd(jointArray(1).theta);
s_j = sind(jointArray(1).theta);

T(1).ij = [c_j, -s_j, 0, 0;...
        s_j, c_j, 0, 0;...
        0, 0, 1, 0;...
        0, 0, 0, 1];

T(1).ji = [c_j, s_j, 0, 0;...
        -s_j, c_j, 0, 0;...
        0, 0, 1, 0;...
        0, 0, 0, 1];
        
for n = 2:numel(jointArray)
    S_j = jointArray(n).S;
    c_j = cosd(jointArray(n).theta);
    s_j = sind(jointArray(n).theta);
    a_ij = linkArray(n-1).a;
    c_ij = cosd(linkArray(n-1).alpha);
    s_ij = sind(linkArray(n-1).alpha);

    T(n).ij = [c_j, -s_j, 0, a_ij;...
            s_j*c_ij, c_j*c_ij, -s_ij, -s_ij*S_j;...
            s_j*s_ij, c_j*s_ij, c_ij, c_ij*S_j;...
            0, 0, 0, 1];

    T(n).ji = [c_j, s_j*c_ij, s_j*s_ij, -c_ij*a_ij;...
            -s_j, c_j*c_ij, c_j*s_ij, s_j*a_ij;...
            0, -s_ij, c_ij, -S_j;...
            0, 0, 0, 1];
end

for n = 1:numel(jointArray)
    T_Fi = eye(4);
    for m = 1:n
        T_Fi = T_Fi*T(m).ij;
    end
    T(n).Fi = T_Fi;
    
    T_Ej = eye(4);
    for m = n:6
        T_Ej = T(m).ji*T_Ej;
    end
    T(n).Ej = T_Ej;
end
tStruct = T;
end