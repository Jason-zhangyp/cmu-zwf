%% Swarm Formation control 
% Description : formation decoding function
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [ formation_out ] = circle2convex( formation_circle, key )
N = length(formation_circle);
center = mean(formation_circle);
%key = zeros(N,2);
%formation_circle = zeros(N,2);
formation_polar = zeros(N,2);
%% remap formation in polar coordinates
for i=1:N %formation 
    dist = sqrt((formation_circle(i,1)-center(1))^2+(formation_circle(i,2)-center(2))^2);
    theta = atan2(formation_circle(i,2)-center(2),(formation_circle(i,1)-center(1)));
    %key(i,:) = [dist,theta];
    formation_polar(i,:) = [dist, theta];
end

%% decoding
dist_real = formation_polar(:,1)./key(:,1);
formation_out = [dist_real.*cos(formation_polar(:,2))+center(1),dist_real.*sin(formation_polar(:,2))+center(2)];

end

