%% Centralized role assignment solving worst-cased assignment problem 
% Description : market-based greedy role assignment algorithm and
% displacement formation control
% Author      : Weifan Zhang 
% Date        : August 12, 2018
% Other Files :

function [poses_out, assignment] = new_role_assignment(poses_in, formation, dt, count, assignment, choose)
    connectivity_radius = 10.0;
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
    
    %% update the assignment
    if mod(count,100)==1
        'enable'
        cost = zeros(N,N);
        for i=1:N %agent
            for  j=1:N %target
                cost(i,j) = sqrt((formation(j,1)-poses_in(i,1)+pose_x-mean_x)^2+(formation(j,2)-poses_in(i,2)+pose_y-mean_y)^2);
            end
        end
        
        if choose==1
            [assignment,~] = munkres(cost);
        elseif choose==2
            %assignment = zeros(1,N);
            assignment_t = assignment;
            cost_t = cost;
            m_inf = N*5;
            record = zeros(1,N);
            stop = 0;
            while stop == 0
                [assignment_t,~] = munkres(cost_t);
                for i=1:N
                   record(i)=cost_t(i,assignment_t(i)); 
                end
                [max_t,~] = max(record);
                if max_t < m_inf
                    for i=1:N %agent
                        for j=1:N %target
                            if cost_t(i,j) >= max_t && cost_t(i,j) < m_inf
                               cost_t(i,j) = m_inf; 
                            end
                        end
                    end
                    assignment = assignment_t;
                    %max_t
                    %cost = cost_t
                else
                    stop = 1;
                end
            end
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


