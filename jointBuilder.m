function joint = jointBuilder(name, mass, length, angle, ~, jointArray, k)
joint.name = name;
joint.id = numel(jointArray)+1;
joint.mass = mass;
joint.S = length;
joint.localVec = [0; 0; joint.S];
joint.theta = angle;
joint.cmVec = joint.localVec/2;
joint.gravForce = joint.mass * k.g * -k.kHat;
end