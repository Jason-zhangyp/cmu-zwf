 %% Swarm Formation Control 
% Description : distributed formation control - Line,circle,square
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [] = demotest(N, t) %N>=5
    %% Initialize formation and start poses
    clc
    dt = 0.01;
    figure
    set(gcf,'Position',[100,100,1000,500], 'color','w') 
    
    rng(38,'twister');
    poses_in = zeros(N,3);
    poses_in(:,1:2) = unifrnd(0,N,[N,2]);
    poses_in(:,3) = unifrnd(0,pi,[N,1]);
    %scatter(poses_in(:,1),poses_in(:,2))
    %hold on 
    %grid on
    
    assert(size(poses_in,2) == 3);
    leg = 3;
    %N = size(poses_in, 1);
    %line
    %formation = zeros(N,2);
    %formation(1:N,1) = (1:N)*3;
    %square:(N=4k)
    
    formation = zeros(N,2);
    formation(1:(N/4),1) = (0:(N/4-1))'*leg;
    formation((N/4+1):(2*N/4),2) = (0:(N/4-1))'*leg;
    formation((N/4+1):(2*N/4),1) = ones(N/4,1)*leg*N/4;
    formation((2*N/4+1):(3*N/4),2) = ones(N/4,1)*leg*N/4;
    formation((2*N/4+1):(3*N/4),1) = (ones(N/4,1)*(N/4+1)-(1:(N/4))')*leg;
    formation((3*N/4+1):N,2) = (ones(N/4,1)*(N/4+1)-(1:(N/4))')*leg;
    
    m=0;
    if m==0
        poses_1 = poses_in;
        poses_2 = poses_in;
        %poses_3 = [formation,poses_in(:,3)];
    else
        poses_1 = [formation,poses_in(:,3)];
        naughty_i = randperm(N,1);
        poses_1(naughty_i,1:2) = unifrnd(N,2*N,[1,2]);
        poses_2 = poses_1;
    end
    
    %%
    ax1 = axes('Position',[0,0,0.5,0.9]);
    ax2 = axes('Position',[0.5,0,0.5,0.9]);
    
    assignment1 = zeros(1,N);
    assignment2 = zeros(1,N);
    
    for i= 1:(t/dt)
        pause(dt/10000)
        
        [poses_1,assignment1] = perfect_formation_control2(poses_1, formation, dt, i, assignment1);
        %[poses_1,assignment1,what] = perfect_formation_control2(poses_1, formation, dt, i, assignment1);
        scatter(ax1,poses_1(:,1),poses_1(:,2),'filled','b')
        grid(ax1,'on')        
        title(ax1,'version2')
        %title(ax1,'With Role Assignment')
                
        %if mod(i,floor(N^0.5/dt))==1
            %assignment2 = assign_init(formation, poses_2);
        %end
        %poses_2 = formation_control(poses_2, formation, dt);
        [poses_2,assignment2] = perfect_formation_control5(poses_2, formation, dt, i, assignment2);
        %[poses_2, assignment2, show] = role_formation_control3(poses_2, formation, dt, assignment2);
        scatter(ax2,poses_2(:,1),poses_2(:,2),'filled','m')  
        grid(ax2,'on')
        title(ax2,'version3')
        %title(ax2,'Without Role Assignment')
        
        %assignment2
        %show
        i
        %if i == 1131
        %    what_out = what;
        %end
    end
    
end

