function [X,dX] = UPDATE_DERIV(acc, Moment, state)
global m Ixx Iyy Izz
% state = [pos vel ang ang_vel]
% dstate = [dpos dvel dang dang_vel]

pos = state(1:3);
vel = state(4:6);
ang = state(7:9);
ang_vel = state(10:12);

% dpos
C_n2b = [1 0 0; 0 cosd(ang(1)) sind(ang(1)); 0 -sind(ang(1)) cosd(ang(1))]...
    *[cosd(ang(2)) 0 sind(ang(2)); 0 1 0; sind(ang(2)) 0 cosd(ang(2))]...
    *[cosd(ang(2)) sind(ang(3)) 0; -sind(ang(3)) cosd(ang(3)) 0; 0 0 1];   % 3-2-1 rotation

dpos = zeros(3,1);
dpos = C_n2b'*vel';

% dvel
P = ang_vel(1);
Q = ang_vel(2);
R = ang_vel(3);
U = vel(1);
V = vel(2);
W = vel(3);

Thrust = acc';  % acc = [ax ay az]', b-frame
g = C_n2b*[0 0 1.63]';
Weight = g;

dvel = -[Q*W-R*V;R*U-R*W;P*V-Q*U] + Thrust + Weight;

% dang
dang = [1 sin(ang(1))*tan(ang(2)) cos(ang(1))*tan(ang(2));...
    0 cos(ang(1)) -sin(ang(1));...
    0 sin(ang(1))*sec(ang(2)) cos(ang(1))*sec(2)]*ang_vel';

% dang_vel
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
dang_vel = I'*[Moment(1) + (Iyy-Izz)*Q*R; Moment(2) + (Izz-Ixx)*R*P;...
    Moment(3) + (Ixx-Iyy)*P*Q];

X = state;
dX = [dpos' dvel' dang' dang_vel'];

end