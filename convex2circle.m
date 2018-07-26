%% Swarm Formation control 
% Description : formation encoding function
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [ formation_circle, key ] = convex2circle( formation_in )
N = length(formation_in);
center = mean(formation_in);
%key = zeros(N,2);
%formation_circle = zeros(N,2);
formation_polar = zeros(N,2);
%% remap formation in polar coordinates
for i=1:N %formation 
    dist = sqrt((formation_in(i,1)-center(1))^2+(formation_in(i,2)-center(2))^2);
    theta = atan2(formation_in(i,2)-center(2),(formation_in(i,1)-center(1)));
    %key(i,:) = [dist,theta];
    formation_polar(i,:) = [dist, theta];
end

%% encoding
radius = min(formation_polar(:,1));
alpha = radius./formation_polar(:,1);
formation_circle = [radius.*cos(formation_polar(:,2))+center(1),radius.*sin(formation_polar(:,2))+center(2)];
key = [alpha, formation_polar(:,2)];

end

