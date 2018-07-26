%% Swarm Formation control 
% Description : distance control for role assignment
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [poses_out, assignment,show] = role_formation_control3 (poses_in, formation, dt, assignment)
    connectivity_radius = 20.0;
    N = size(poses_in, 1);
    gain_v = 1/N;
    gain_w = 2;

    assert(size(poses_in,2) == 3);
    assert(size(formation,1) == size(poses_in,1));
            
    L = N*diag(ones(1,N)) - ones(N,N);
    
    poses_out = poses_in;
    pose_x = mean(poses_in(:,1));
    pose_y = mean(poses_in(:,2));
    formation_x = mean(formation(:,1));
    formation_y = mean(formation(:,2));
    
    %if mod(count,floor(N^0.5/dt))==1
        %% preparation
        cost = zeros(N,N);

        target_hist = zeros(N,N+1);
        %agent_role = zeros(N,2);
       
        for i=1:N %agent_role
            target_hist(assignment(i),1) = target_hist(assignment(i),1)+1;
            target_hist(assignment(i),target_hist(assignment(i),1)+1) = i;
        end

        %% assign roles
        for i=1:N %agent
            %dist_t = ones(1,N)*999;
            dist_t = zeros(1,N);
            for j=2:(target_hist(assignment(i),1)+1)
               poses_t = poses_in(target_hist(assignment(i),j),1:2);
               dist_t(target_hist(assignment(i),j)) = sqrt((poses_t(1)-formation(assignment(i),1)-pose_x+formation_x)^2+(poses_t(2)-formation(assignment(i),2)-pose_y+formation_y)^2); 
            end
            [~,max_i] = max(dist_t);
            if i~=max_i %min_i
                role_left = mod(assignment(i)-1,N);
                role_right = mod(assignment(i)+1,N);
                if role_left==0
                    role_left = N;
                end
                if role_right==0
                    role_right = N;
                end
                flag_t = target_hist(role_right,1)-target_hist(role_left,1);
                if flag_t == 0
                    if cost(i,role_right)>cost(i,role_left)
                        assignment(i) = role_left;
                    else
                        assignment(i) = role_right;
                    end
                elseif flag_t > 0
                    assignment(i) = role_left;
                else
                    assignment(i) = role_right;
                end
            end
        end

        %assignment = agent_role(:,2)';
        show = target_hist(:,1)';
    %end
    
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

