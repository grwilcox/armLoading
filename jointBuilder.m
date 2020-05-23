% Initializes all characteristics of a joint independent of the parameters
% of the rest of the system (i.e. anything that doesn't need transforms)
% 
% name - string, plain identifier for joint
% id - int, unique numerical identifier for joint, the i of S_i
% length - distance S_j between link vectors a_ij and a_jk
% mass - mass of all parts comoving with reference frame [kg]
% CoM - center of mass displacement from origin of reference frame
% Inertia - (CoM) moment of inertia matrix of all elements in frame
% axes - mapping from [x,y,z] axes of CoM and inertia (from CAD) to axes of
% joint reference frame. link vector a is x', joint vector S is z'
% angle - skew angle theta_j between link vectors a_ij and a_jk
% velocity - angular velocity of joint
% acceleration - angular acceleration of joint
% k - struct of gravity parameters
%
% joint - struct of characteristics of the single joint at hand

function joint = jointBuilder(name, id, length, mass, CoM,...
    Inertia, axes, angle, velocity, acceleration, k)
joint.name = name;
joint.id = id;
joint.S = length*k.m_in;
joint.mass = mass;
joint.cmLocal = [CoM(axes(1)); CoM(axes(2)); CoM(axes(3))];
joint.Inertia = parallelAxis(Inertia,CoM,mass,axes);
joint.theta = angle;
joint.velocity = velocity;
joint.acceleration = acceleration;
joint.localVec = [0; 0; joint.S];
joint.gravForce = joint.mass * k.g * -k.kHat;
end
