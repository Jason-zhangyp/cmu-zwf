%% Swarm Formation control 
% Description : distance control for role assignment
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [poses_out, assignment, show] = role_formation_control2(poses_in, formation, dt, count, assignment, show)
    connectivity_radius = 20.0;
    gain_v = 0.8;
    gain_w = 0.8;

    assert(size(poses_in,2) == 3);
    assert(size(formation,1) == size(poses_in,1));
    N = size(poses_in, 1);
        
    L = N*diag(ones(1,N)) - ones(N,N);
    
    poses_out = poses_in;
    pose_x = mean(poses_in(:,1));
    pose_y = mean(poses_in(:,2));
    formation_x = mean(formation(:,1));
    formation_y = mean(formation(:,2));
    
    if mod(count,floor(N^0.5/dt))==1
        %% preparation
        cost = zeros(N,N);

        target_hist = zeros(N,N+1);
        agent_role = zeros(N,2);
        for i=1:N %agent
            for j=1:N %target
                cost(i,j) = sqrt((poses_in(j,1)-formation(i,1)-pose_x+formation_x)^2+(poses_in(j,2)-formation(i,2)-pose_y+formation_y)^2);
            end
        end

        for i=1:N %agent
            [agent_role(i,1),agent_role(i,2)] = min(cost(i,:));
        end

        for i=1:N %agent_role
            target_hist(agent_role(i,2),1) = target_hist(agent_role(i,2),1)+1;
            target_hist(agent_role(i,2),target_hist(agent_role(i,2),1)+1) = i;
        end

        %% assign roles
        for i=1:N %agent
            dist_t = ones(1,N)*999;
            for j=2:(target_hist(agent_role(i,2),1)+1)
               dist_t(target_hist(agent_role(i,2),j)) = agent_role(target_hist(agent_role(i,2),j),1); 
            end
            [~,min_i] = min(dist_t);
            if i~=min_i
                role_left = mod(agent_role(i,2)-1,N);
                role_right = mod(agent_role(i,2)+1,N);
                if role_left==0
                    role_left = N;
                end
                if role_right==0
                    role_right = N;
                end
                flag_t = target_hist(role_right,1)-target_hist(role_left,1);
                if flag_t == 0
                    if cost(i,role_right)>cost(i,role_left)
                        agent_role(i,2) = role_left;
                    else
                        agent_role(i,2) = role_right;
                    end
                elseif flag_t > 0
                    agent_role(i,2) = role_left;
                else
                    agent_role(i,2) = role_right;
                end
            end
        end

        assignment = agent_role(:,2)';
        show = target_hist(:,1)';
    end
    %% controller
    z = zeros(N,2);
    for i=1:N
        z(i,:) = formation(assignment(i),:);
    end
    
    v = -L*(poses_in(:,1:2)-z);
    
    for i=1:N
        heading_i = poses_in(i, 3);            
                        
        dtheta = atan2(v(i,2), v(i,1)) - heading_i;
        w = atan2(sin(dtheta), cos(dtheta));
        
        %b = [cos(heading_i); sin(heading_i)];
        u_v = gain_v * sqrt(v(i,1)^2+v(i,2)^2);
        u_w = gain_w * w;
        
        poses_out(i,:) = robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

