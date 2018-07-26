%% Swarm Formation control 
% Description : distance control for role assignment
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [poses_out,role_index] = role_formation_control(poses_in, formation, dt, role_index,isFirst)
    connectivity_radius = 20.0;
    gain_v = 0.8;
    gain_w = 0.8;

    assert(size(poses_in,2) == 3);
    assert(size(formation,1) == size(poses_in,1));
    N = size(poses_in, 1);
    
    poses_out = poses_in;
    %if isFirst==1
        %% obtain cost function
        cost = zeros(N,N);
        pose_x_mean = mean(poses_in(:,1));
        pose_y_mean = mean(poses_in(:,2));
        formation_x_mean = mean(formation(:,1));
        formation_y_mean = mean(formation(:,2));
        for i=1:N %formation
            for j=1:N %agents
                cost(i,j) = sqrt((poses_in(j,1)-formation(i,1)-pose_x_mean+formation_x_mean)^2+(poses_in(j,2)-formation(i,2)-pose_y_mean+formation_y_mean)^2);
            end
        end

        %% assign roles
        cost = floor(cost*10)
        role_index = match(cost);
    %end
    %% distance controller
    for i=1:N
        position_i = poses_in(i, 1:2)';
        heading_i = poses_in(i, 3);
                      
        v = zeros(2, 1);
        w = 0;
        n = 0;
        for j=1:N
            if i==j, continue; end
            position_j = poses_in(j, 1:2)';
            heading_j = poses_in(j, 3);
            
            d = position_j - position_i;
            if norm(d,2) < connectivity_radius
                dv = d - (formation(role_index(j),:)' - formation(role_index(i),:)');
                n = n + 1;
            else 
                dv = 0;
            end
            v = v + dv;
        end
        v = v / (n + 1);
        dtheta = atan2(v(2), v(1)) - heading_i;
        w = atan2(sin(dtheta), cos(dtheta));
        
        b = [cos(heading_i); sin(heading_i)];
        u_v = gain_v * v' * b;
        u_w = gain_w * w;
        
        poses_out(i,:) =  robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

