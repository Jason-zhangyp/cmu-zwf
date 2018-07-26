%% Swarm Formation control 
% Description : distance control for role assignment
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [poses_out] = perfect_formation_control(poses_in, formation, dt)
    %connectivity_radius = 20.0;
    gain_v = 1.0;
    gain_w = 1.2;

    assert(size(poses_in,2) == 3);
    assert(size(formation,1) == size(poses_in,1));
    N = size(poses_in, 1);
    
    poses_out = poses_in;
    
    %% update assignment
    assignment = zeros(1,N);
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
                if target_check(j)==0
                    distance(i) = distance(i) + sqrt((formation(j,1)-poses_in(i,1))^2+(formation(j,2)-poses_in(i,2))^2);
                end
            end
        end
        [max_t,max_i]=max(distance);%subscript of the farthest agent
        agent_check(max_i) = 1;

        %% find the nearest target of the agent
        dist_r = -1;
        %dist_t = 0;
        for i=1:N %targets
            if target_check(i)==1
                continue
            end
            dist_t = sqrt((formation(i,1)-poses_in(max_i,1))^2+(formation(i,2)-poses_in(max_i,2))^2);
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
    assignment
        
    %% controller
    for i=1:N
        position_i = poses_in(i, 1:2)';
        heading_i = poses_in(i, 3);            
        
        w=0;
        v=zeros(2,1);
        v=[formation(assignment(i),1)-position_i(1);formation(assignment(i),2)-position_i(2)];
        
        dtheta = atan2(v(2), v(1)) - heading_i;
        w = atan2(sin(dtheta), cos(dtheta));
        
        %b = [cos(heading_i); sin(heading_i)];
        u_v = gain_v * sqrt(v(1)^2+v(2)^2);
        u_w = gain_w * w;
        
        poses_out(i,:) = robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

