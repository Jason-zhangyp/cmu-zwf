%% Swarm Formation Control 
% Description : distance control - Line,(circle),square
% Author      : Weifan Zhang 
% Date        : February 7, 2018
% Other Files :

function [] = demo1(poses_in,N, t)
    clc
    dt = 0.1;
    %poses_in = zeros(N,3);
    %poses_in(:,1:2) = unifrnd(0,20,[N,2]);
    %poses_in(:,3) = unifrnd(0,pi,[N,1]);
    scatter(poses_in(:,1),poses_in(:,2))
    hold on 
    grid on
    
    assert(size(poses_in,2) == 3);
    %N = size(poses_in, 1);
    %line
    formation = zeros(N,2);
    formation(1:N,1) = (1:N)*3;
    %square:N=4k
    %formation = zeros(N,2);
    %formation(1:(N/4),1) = (0:(N/4-1))'*3;
    %formation((N/4+1):(2*N/4),2) = (0:(N/4-1))'*3;
    %formation((N/4+1):(2*N/4),1) = ones(N/4,1)*3*N/4;
    %formation((2*N/4+1):(3*N/4),2) = ones(N/4,1)*3*N/4;
    %formation((2*N/4+1):(3*N/4),1) = (ones(N/4,1)*(N/4+1)-(1:(N/4))')*3;
    %formation((3*N/4+1):N,2) = (ones(N/4,1)*(N/4+1)-(1:(N/4))')*3;
    
    
    
    for i= 1:(t/dt)
        pause(0.1)
        poses_out = formation_control(poses_in, formation, dt);
        scatter(poses_out(:,1),poses_out(:,2))
        poses_in = poses_out;
    end
    
end

