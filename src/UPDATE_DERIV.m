function [X,dX] = UPDATE_DERIV(Moment, acc, state)
global m Ixx Iyy Izz
% state = [pos vel ang ang_vel]
% dstate = [dpos dvel dang dang_vel]
pos = state(1:3);
vel = state(4:6);
ang = state(7:9);
ang_vel = state(10:12);

% dpos
% C_n2b = [1 0 0; 0 cos(ang(1)) sin(ang(1)); 0 -sin(ang(1)) cos(ang(1))]...
%     *[cos(ang(2)) 0 -sin(ang(2)); 0 1 0; sin(ang(2)) 0 cos(ang(2))]...
%     *[cos(ang(2)) sin(ang(3)) 0; -sin(ang(3)) cos(ang(3)) 0; 0 0 1];   % 3()-2-1
C_b2n = [cos(ang(2))*cos(ang(3)) cos(ang(2))*sin(ang(3)) -sin(ang(2));...
    sin(ang(1))*sin(ang(2))-cos(ang(1))*sin(ang(3)) sin(ang(1))*sin(ang(2))*sin(ang(3))+cos(ang(1))*cos(ang(3)) sin(ang(1))*cos(ang(2));...
    cos(ang(1))*sin(ang(2))*cos(ang(3))+sin(ang(1))*sin(ang(3)) cos(ang(1))*sin(ang(2))*sin(ang(3))-sin(ang(1))*cos(ang(3)) cos(ang(1))*cos(ang(2))]';

dpos = C_b2n*vel';

% dvel
P = ang_vel(1);
Q = ang_vel(2);
R = ang_vel(3);
U = vel(1);
V = vel(2);
W = vel(3);

%Thrust = C_b2n'*acc';  % acc = [ax ay az]', b-frame
Thrust = [0 0 -norm(acc)]';  
g = C_b2n'*[0 0 1.63]';
Weight = g;

dvel = -[Q*W-R*V;R*U-P*W;P*V-Q*U] + Thrust + Weight;

% dang
dang = [1 sin(ang(1))*tan(ang(2)) cos(ang(1))*tan(ang(2));...
    0 cos(ang(1)) -sin(ang(1));...
    0 sin(ang(1))*sec(ang(2)) cos(ang(1))*sec(ang(2))]*ang_vel';

% dang_vel
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
dang_vel = inv(I)*[Moment(1) + (Iyy-Izz)*Q*R; Moment(2) + (Izz-Ixx)*R*P;...
    Moment(3) + (Ixx-Iyy)*P*Q];

X = state;
dX = [dpos' dvel' dang' dang_vel'];

end