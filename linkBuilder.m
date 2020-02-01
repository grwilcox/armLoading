function link = linkBuilder(name, mass, length, angle, linkArray, ~, k)
link.name = name;
link.id = numel(linkArray)+1;
link.mass = mass;
link.a = length;
link.localVec = [link.a; 0; 0];
link.alpha = angle;
link.cmVec = link.localVec/2;
link.gravForce = link.mass * k.g * -k.kHat;
end