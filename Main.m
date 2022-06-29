clear all; close all;
global m Ixx Iyy Izz roll0 dt t_go
global K_phi K_P K_theta K_Q K_psi K_R

h = 15000;

Ixx = 2500;
Iyy = 4500;
Izz = 350;

K_phi = 2450;
K_P = 3450;
K_theta = 4450;
K_Q = 6000;
K_psi = 320;
K_R = 460;


%% INITIALIZE STATE
pos = [10000 10000 -h];   % n-frame
vel = [1300 1300 0];
ang = [0 0 0];   % rad
ang_vel = [0 0 0];
roll0 = 0;

m = 2100;  % mass of lunar-lander, kg

initial_state = [pos vel ang ang_vel];  % I-frame
state_t = [0 0 0 0 0 0];    % I-frame


%% SIMULATION
dt = 1;

i = 1;
t_go = 30;
tic

[acc,t_go] = guidance_moon(initial_state,state_t);  % n-frame
state = initial_state; 
z = state(3);

% acc = guidance_moon(state,state_t);

while i <= t_go/dt
    save_state(i,:) = state;  % n-frame
    
    % 6DOF - RK4
    Num = 1;
    save_s = 0;
    sum_k = 0;
    time = 0;

    [acc(i,:),t_go] = guidance_moon(state,state_t);
%     acc(i,:) = [1 -1 0];
%     t_go_buffer(i) = t_go;
%     acc_buffer(i,:) = acc;
    
    angle_cmd(i,:) = acc2angle(acc(i,:));
    Moment_cmd(i,:) = attitude_controller(save_state(i,:),angle_cmd(i,:))';
    
%     Moment_cmd(i,:) = [0 0 0];
%     Thrust(i,:) = m*Rotate_vec(acc(i,:)', angle_cmd(i,:), 'n2b')';
% %     [T,Moment(i,:),Thrust(i,:)] = Cal_thrust(m*acc(i,:), angle_cmd(i,:), Moment_cmd(i,:));
    while(Num < 5)
        [X,dX] = UPDATE_DERIV(Moment_cmd(i,:), m*acc(i,:), state);
        [X,time,save_s,sum_k,Num] = RK4(X,dX,save_s,sum_k,time,Num,dt);
    end
    state = X;
    state(7:9) = ang_limit(state(7:9));  % ang_limitation (0 <= ang < 2*pi)
    x = state(1);
    y = state(2);
    z = state(3);

    i = i + 1;
end