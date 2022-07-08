global m t_go MOI K  K_phi K_P K_theta K_Q K_psi K_R

% Initial states of lunar lander
pos = [100 100 -1500];   % Position, I-frame
ang = [0 0 0];         % Euler angle, rad
vel_I = [-25 0 80];    % Velocity, I-frame, m/s
ang_vel = [0 0 0];     % Angular velocity, rad/s
vel = (DCM_b2n(ang)'*vel_I')';
initial_state = [pos vel ang ang_vel];  % I-frame

% initial states of target object(point)
% state_t = [x y z vz vy vz]
state_t = [0 0 0 0 0 0];    % I-frame

% initial mass of lunar lander
m = 2100;  % mass of lunar-lander, kg

% Moment of Inertia of lunar-lander
Ixx = 2500;
Iyy = 4500;
Izz = 350;
MOI = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

% constants K of Attitude controller
%%% angular velocity constants
K_phi = 62500;
K_theta = 112500;
K_psi = 8750;
K.ang = [K_phi K_theta K_psi];
%%% Euler angle constants
K_P = 21250;
K_Q = 38250;
K_R = 2975;
K.ang_vel = [K_P K_Q K_R];

