function [V] = potential_field(h, obs)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
i = size(obs,1);
j = size(obs,2);
V = [];
for k = 1:1:j
    aux = exp(distance(h,obs(:,k)));
    disp(aux);
    pause(1)
    V = [V;aux];
end

end

