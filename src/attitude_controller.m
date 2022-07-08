% function Moment_cmd = attitude_controller(state,angle_cmd)
% global K
% 
% angle = state(7:9);
% ang_vel = state(10:12);
% Moment_cmd = -K.ang_vel.*ang_vel + K.ang.*(angle_cmd - angle);

function Moment_cmd = attitude_controller(state,angle_cmd)
global K_phi K_P K_theta K_Q K_psi K_R

phi = state(7);
theta = state(8);
psi = state(9);

P = state(10);
Q = state(11);
R = state(12);

phi_cmd = angle_cmd(1);
theta_cmd = angle_cmd(2);
psi_cmd = angle_cmd(3);

Moment_cmd(1,1) = -K_P*P + K_phi*(phi_cmd - phi);
Moment_cmd(1,2) = -K_Q*Q + K_theta*(theta_cmd - theta);
Moment_cmd(1,3) = -K_R*R + K_psi*(psi_cmd - psi);

end


