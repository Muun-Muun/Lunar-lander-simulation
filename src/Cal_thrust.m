function [t,m,Thrust] = Cal_thrust(acc, ang, Moment)
global m
%thrust(1,1)=T_1;
%t(2,1)=T_2;
%t(3,1)=T_3;
%t(4,1)=T_4;
Thrust = m*acc;
C_n2b = -[1 0 0; 0 cos(ang(1)) sin(ang(1)); 0 -sin(ang(1)) cos(ang(1))]...
    *[cos(ang(2)) 0 -sin(ang(2)); 0 1 0; sin(ang(2)) 0 cos(ang(2))]...
    *[cos(ang(3)) sin(ang(3)) 0; -sin(ang(3)) cos(ang(3)) 0; 0 0 1];   % 3-2-1 rotation
F = C_n2b*Thrust'; % b-frame
F_z = F(3);
X = [F_z; Moment(1); Moment(2); Moment(3)];

%% 달착륙선의 크기 및 노즐의 고정 각도 // a: 고정각도, dx&dy&dz : 정육면체 한변의 절반 길이
a = pi/6;
dy = 0.5;
dx = 0.5;
dz = 1;
%% 커멘드 힘과 모멘트 F_z, M_x, M_y, M_z // t = [T_1, T_2, T_3, T_4]^T

A = [ cos(a), cos(a), cos(a), cos(a); 
    dy, -dy, -dy, dy; 
    sin(a).*dz-cos(a).*dx, sin(a).*dz-cos(a).*dx, cos(a).*dx-sin(a).*dz, cos(a).*dx-sin(a).*dz; 
    -dy, dy,-dy, dy];

%t = solve(A,X);
%t= X/A;
t= inv(A)*X;



%% LMN 계산. m = [L M N]^T

B = [ dy, -dy, -dy, dy; 
    sin(a).*dz-cos(a).*dx, sin(a).*dz-cos(a).*dx, cos(a).*dx-sin(a).*dz, cos(a).*dx-sin(a).*dz; 
    -dy, dy,-dy, dy];

m = B*t;

%% fx,fy,fz 계산. f = [F_x F_y F_z]^T

% C = [ sin(a), sin(a), -sin(a), -sin(a); 
%     0, 0, 0, 0; 
%     cos(a), cos(a), cos(a), cos(a)];
% 
% f = C*t;

end