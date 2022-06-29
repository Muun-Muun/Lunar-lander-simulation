function guidance_moon_only
global dt T n x_des lamda2 lamda lb ub k2_max k_max lr A_tilde b_tilde epsilon point t_go
T =50;
n = 50;
k_max = 2000;
k2_max = 100;

dt = T/n ;
x_des = zeros(6,1);
x_0 = [0 0 -1500 -0 -0 -10]';

lamda = 1;
lamda2 = 100;
lr = 0.01;

lb = 0;
ub = 5;
epsilon = 0.00001;
ts = linspace(0,T,n+1);

A = eye(6);
B = zeros(6,3);
for i = 1:3
    A(i,i+3)= dt;
    B(i,i)=dt^2/2;
    B(i+3,i)= dt;
end
x = zeros(6,n+1);
x(:,1) = x_0;
G = zeros(6,3*n);
P = zeros(6,6*n);
g = 1.63 ;
%b = [0, 0, -0.5*g*dt^2, 0, 0, -g*dt]';
b = [0, 0, 0.5*g*dt^2, 0, 0, g*dt]';
bs = repmat(b, n, 1);
for i = 1: n       
    G(:,3*i-2:3*i) = A^(n-i)*B;
end
for i = 1: n
    P(:,6*i-5:6*i)= A^(n-i);
end

Q = P*bs;
first = A^n*x_0;
u_tilde = lsqr(G, x_des - first-Q);
u = reshape(u_tilde,[3,n]);   %plot
% figure(1)
% plot(ts(1:end-1),u(1,:))


for i = 1:n
  x(:,i+1)=A*x(:,i)+B*u(:,i)+b;
end     

G_tilde = vertcat(G(1:3,:),lamda2*G(4:6,:));
xqf = x_des-Q-first;
xqf_tilde = zeros(6,1);
xqf_tilde(1:3) = xqf(1:3);
xqf_tilde(4:6) = lamda2*xqf(4:6);
A_tilde = vertcat(G_tilde,sqrt(lamda)*eye(3*n));
b_tilde = vertcat(xqf_tilde,zeros(3*n,1));    

gradients(u_tilde);
% end

%plot

u = reshape(point, [3,n]);
ccc = point
u_temp = zeros(n,1);
for i=1: n
    u_temp(i) = norm(u(:,i));
end
figure()
plot(ts(1:end-1), u_temp)
ylim([0,8])
figure()
plot3(x(1,:),x(2,:), x(3,:))
figure()
plot(ts(1:end-1),u(1,:))

end