clear all

specs ={"a12 >> base?", 1, 3*pi/2, "S1 >> base rotate", 0,...
            1.162, [0;-.015;-.021]-[0;-.087;0],...
            [.008,0,0;0,.003,-.002;0,-.002,.007],[1,3,2];...
        "a23 >> upper arm CF", 13.55, 0,"S2 >> shoulder", 1,...
            1.543, [0;.005;-.016]-[0;-13.55*.0254;-.022],...
            [2.139,0,0;0,1.9,.139;0,.139,1.475]/1e3,[2,1,3];...
        "a34 >> elbow complex", 1, pi/2, "S3 >> elbow", -3.564,...
            .806, [.002;-.023;0]-[0;-.037;0],...
            [1.237,.06,.003;.06,.521,.012;.003,.012,1.277]/1e3,[1,2,3];...
        "a45 >> elbow complex 2: kinematic boogaloo", 1, 3*pi/2, "S4 >> wrist R1", 15.159,...
            .436, [.023;.068;0]-[0;-11.77*.0254;0],...
            [1.889,-.648,0;-.648,.457,0;0,0,2.213]/1e3,[2,3,1];...
        "a56 >> wrist complex", 1, pi/2,"S5 >> wrist T", 1,...
            .738, [0;.068;.013]-[0;0;.013],...
            [5.122,.007,.004;.007,.458,-.64;.004,-.64,5.009]/1e3,[1,2,3];...
        "a67 >> EE orient", -1, [], "S6 >> wrist R2", 5.876,...
            .392, [-.001;.003;.064]-[-.008;.008;-.153],...
            [2.671,-.002,.003;-.002,2.707,-.17;.003,-.17,.235]/1e3,[1,2,3];}; % S_6 actually 11.876 ?

[links, joints, T, R] = initArm(specs);

qtest = [pi/6; 3*pi/2+18*(pi/180); pi/2+80*(pi/180);...
            25*(pi/180); 45*(pi/180); -35*(pi/180)];
plotArm(links,joints,qtest)

[q0t,q1t,q2t,tt] = parseIKoutput("arm_ik_output.txt");
loads = getJointLoads(joints,q0t,q1t,q2t);
plotLoads(loads,tt,joints,5)
plotLoads(loads,tt,joints,'total')
