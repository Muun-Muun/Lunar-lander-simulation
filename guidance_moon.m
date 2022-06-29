function [acc_n,t_go] = guidance_moon(state,state_t)
global dt t_go

% state = [x_I y_I z_I U_b V_b W_b roll pitch yaw P Q R]
% 
R = norm(state_t(1:3) - state(1:3))+100;
Vc = norm(state_t(4:6) - state(4:6));
t_go = R/Vc;
n = floor(t_go/dt);
A = eye(6);
B = zeros(6,3);

for i = 1 : 3
  A(i,i+3) = dt;
  B(i,i) = dt^2/2;
  B(i+3,i) = dt;
end

G = zeros(6,3*n);
P = zeros(6,6*n);
g = 1.63;
b = [0, 0, 0.5*g*dt^2, 0, 0, g*dt];
bs = repmat(b,1,n)';
for i = 1 : n
  G(:,3*i-2:3*i) = A^(n-i)*B;
  P(:,6*i-5:6*i)= A^(n-i);
  Q = P*bs;
end

% Rotate vel_b -> vel_n
ang = state(7:9);
state(4:6) = Rotate_vec(state(4:6)', ang, 'b2n')';  % row vec
 
first = A^n*state(1:6)';
u_temp = lsqr(G, state_t' - first - Q );
acc_temp = reshape(u_temp,[3,n]);
% acc_I = acc_temp(:,1)'; % I-frame, column vec
% acc_n = Rotate_vec(acc_I, [pi 0 0], 'I2n')';  % n-frame, row vec
acc_n = acc_temp(:,1)';
% acc_n = acc_temp';

end