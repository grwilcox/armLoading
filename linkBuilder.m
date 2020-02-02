% Initializes all characteristics of a link independent of the parameters
% of the rest of the system (i.e. anything that doesn't need transforms)
% 
% name - string, plain identifier for link
% id - int, unique numerical identifier for link, the i of a_ij
% mass - mass of component in kg
% length - distance a_ij between joint vectors S_i and S_j
% angle - skew angle alpha_ij between joint vectors S_i and S_j
% k - struct of gravity parameters
%
% link - struct of characteristics of the single link at hand


function link = linkBuilder(name, id, mass, length, angle, k)
link.name = name;
link.id = id;
link.mass = mass;
link.a = length;
link.localVec = [link.a; 0; 0];
link.alpha = angle;
link.cmLocal = link.localVec/2; % TODO: add cmLocal input argument
link.gravForce = link.mass * k.g * -k.kHat;
end
