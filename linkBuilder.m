% Initializes all characteristics of a link independent of the parameters
% of the rest of the system (i.e. anything that doesn't need transforms)
% 
% name - string, plain identifier for link
% id - int, unique numerical identifier for link, the i of a_ij
% length - distance a_ij between joint vectors S_i and S_j
% angle - skew angle alpha_ij between joint vectors S_i and S_j
% k - struct of gravity parameters
%
% link - struct of characteristics of the single link at hand

function link = linkBuilder(name, id, length, angle, k)
link.name = name;
link.id = id;
link.a = length*k.m_in;
link.localVec = [link.a; 0; 0];
link.alpha = angle;
end
