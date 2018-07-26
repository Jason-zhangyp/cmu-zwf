%% Swarm Formation control 
% Description : distance control for role assignment
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [poses_out, assignment] = perfect_formation_control2(poses_in, formation, dt, count, assignment)
    %connectivity_radius = 40.0;
    N = size(poses_in, 1);
    gain_v = 1/N;
    gain_w = 2;
    
    assert(size(poses_in,2) == 3);
    assert(size(formation,1) == size(poses_in,1));
    
    L = N*diag(ones(1,N)) - ones(N,N);

    poses_out = poses_in;
    mean_x = mean(formation(:,1));
    mean_y = mean(formation(:,2));
    pose_x = mean(poses_in(:,1));
    pose_y = mean(poses_in(:,2));
    %what = -1;
    %% update assignment
    if mod(count,floor(N^0.5/dt))==1
        %assignment = zeros(1,N);
        %what = zeros(N,2);
        rest_num = N;
        agent_check = zeros(1,N);
        target_check = zeros(1,N);
        while rest_num>=1
            %% find the farthest agents of all targets
            distance = zeros(1,N);
            for i=1:N %agents
                if agent_check(i)==1
                    continue
                end
                for j=1:N %targets
                    if target_check(j)==0%||count == 1131
                        distance(i) = distance(i) + sqrt((formation(j,1)-poses_in(i,1)+pose_x-mean_x)^2+(formation(j,2)-poses_in(i,2)+pose_y-mean_y)^2);
                    end
                end
            end
            [max_t,max_i]=max(distance);%subscript of the farthest agent
             if max_t < 1 && rest_num==N
                break
            end
            agent_check(max_i) = 1;
            %what(N-rest_num+1,:)=[max_i,max_t];
    
            %if
            
            %% find the nearest target of the agent
            dist_r = -1;
            %dist_t = 0;
            for i=1:N %targets
                if target_check(i)==1
                    continue
                end
                dist_t = sqrt((formation(i,1)-poses_in(max_i,1)+pose_x-mean_x)^2+(formation(i,2)-poses_in(max_i,2)+pose_y-mean_y)^2);
                if dist_r<0||dist_r>dist_t
                   dist_r = dist_t;
                   assignment(max_i) = i;
                end
            end
            %max_i
            %assignment(max_i)
            target_check(assignment(max_i)) = 1;
            rest_num = rest_num-1;
        end
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

