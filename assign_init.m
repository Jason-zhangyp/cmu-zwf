%% Swarm Formation control 
% Description : assign roles for the first time
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [ assignment ] = assign_init( formation, poses_in )
    N = length(formation);
    pose_x = mean(poses_in(:,1));
    pose_y = mean(poses_in(:,2));
    formation_x = mean(formation(:,1));
    formation_y = mean(formation(:,2));

    cost = zeros(N,N);
    assignment = zeros(1,N);
    for i=1:N %agent
        for j=1:N %target
            cost(i,j) = sqrt((poses_in(j,1)-formation(i,1)-pose_x+formation_x)^2+(poses_in(j,2)-formation(i,2)-pose_y+formation_y)^2);
        end
    end

    for i=1:N %agent
        [~,assignment(i)] = min(cost(i,:));
    end

end


