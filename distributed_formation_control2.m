%% Swarm Formation control 
% Description : distance control for role assignment
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [poses_out] = distributed_formation_control2(poses_in, formation, dt)
    %connectivity_radius = 20.0;
    gain_v = 0.8;
    gain_w = 1.2;

    assert(size(poses_in,2) == 3);
    assert(size(formation,1) == size(poses_in,1));
    N = size(poses_in, 1);
    
    poses_out = poses_in;
    
    %% find the five nearest targets for every agent
    aim = zeros(N,5);
    distance = ones(N,5)*(-1);
    for i=1:N %agents
        for j=1:N %targets
            dist_t = sqrt((formation(j,1)-poses_in(i,1))^2+(formation(j,2)-poses_in(i,2))^2);
            if distance(i,1)<0
                aim(i,1)=j;
                distance(i,1) = dist_t;
            elseif distance(i,1)>dist_t
                aim(i,:)=[j,aim(i,1:4)];
                distance(i,:)=[dist_t,distance(i,1:4)];
            elseif distance(i,2)<0
                aim(i,2)=j;
                distance(i,2) = dist_t;
            elseif distance(i,2)>dist_t
                aim(i,2:5)=[j,aim(i,2:4)];
                distance(i,2:5)=[dist_t,distance(i,2:4)];
            elseif distance(i,3)<0
                aim(i,3)=j;
                distance(i,3) = dist_t;
            elseif distance(i,3)>dist_t
                aim(i,3:5)=[j,aim(i,3:4)];
                distance(i,3:5)=[dist_t,distance(i,3:4)];
            elseif distance(i,4)<0
                aim(i,4)=j;
                distance(i,4) = dist_t;
            elseif distance(i,4)>dist_t
                aim(i,4:5)=[j,aim(i,4)];
                distance(i,4:5)=[dist_t,distance(i,4)];
            elseif distance(i,5)<0||distance(i,5)>dist_t
                aim(i,5)=j;
                distance(i,5) = dist_t;
            end
        end
    end

    %% obtain the density histogram for every target
    histogram = zeros(1,N);
    for i=1:N
       histogram(aim(i,1))=histogram(aim(i,1))+1;
    end
    aim
    distance
    histogram
    
    %% controller
    for i=1:N
        position_i = poses_in(i, 1:2)';
        heading_i = poses_in(i, 3);            
        
        w=0;
        v=zeros(2,1);
        %alpha = atan2(formation(aim(i,1),2)-position_i(2),formation(aim(i,1),1)-position_i(1));
        %v=[cos(alpha);sin(alpha)];
        v=[formation(aim(i,1),1)-position_i(1);formation(aim(i,1),2)-position_i(2)];
        
        dv=zeros(2,1);
        if aim(i,1)-1>0
            dv = dv + distance(i,1)*(histogram(aim(i,1))-histogram(aim(i,1)-1))*[formation(aim(i,1)-1,1)-formation(aim(i,1),1);formation(aim(i,1)-1,2)-formation(aim(i,1),2)];
        end
        if aim(i,1)+1<=N
            dv = dv + distance(i,1)*(histogram(aim(i,1))-histogram(aim(i,1)+1))*[formation(aim(i,1)+1,1)-formation(aim(i,1),1);formation(aim(i,1)+1,2)-formation(aim(i,1),2)];
        end
        
        v = v+0.3*dv;
        dtheta = atan2(v(2), v(1)) - heading_i;
        w = atan2(sin(dtheta), cos(dtheta));
        
        %b = [cos(heading_i); sin(heading_i)];
        u_v = gain_v * sqrt(v(1)^2+v(2)^2);
        u_w = gain_w * w;
        
        poses_out(i,:) = robot_model(poses_in(i,:), u_v, u_w, dt);
    end
end

