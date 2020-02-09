% Initializes all characteristics of a joint independent of the parameters
% of the rest of the system (i.e. anything that doesn't need transforms)
% 
% name - string, plain identifier for joint
% id - int, unique numerical identifier for joint, the i of S_i
% mass - mass of component in kg
% length - distance S_j between link vectors a_ij and a_jk
% angle - skew angle theta_j between link vectors a_ij and a_jk
% k - struct of gravity parameters
%
% joint - struct of characteristics of the single joint at hand

function joint = jointBuilder(name, id, mass, length, angle, k)
joint.name = name;
joint.id = id;
joint.mass = mass;
joint.S = length*k.m_in;
joint.localVec = [0; 0; joint.S];
joint.theta = angle;
joint.cmLocal = joint.localVec/2;   % TODO: add cmLocal input argument
joint.gravForce = joint.mass * k.g * -k.kHat;
end
