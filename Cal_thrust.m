function [t,m] = Cal_thrust(F_z, Moment)

%thrust(1,1)=T_1;
%t(2,1)=T_2;
%t(3,1)=T_3;
%t(4,1)=T_4;
M_x = Moment(1);
M_y = Moment(2);
M_z = Moment(3);
X = [F_z; M_x; M_y; M_z];

%% 달착륙선의 크기 및 노즐의 고정 각도
a = pi/6;
dy = 114.25;
dx = 114.25;
dz =114.25;
%% 커멘드 힘과 모멘트 F_z, M_x, M_y, M_z

A = [ cos(a), cos(a), cos(a), cos(a); 
    dy, -dy, -dy, dy; 
    sin(a).*dz-cos(a).*dx, sin(a).*dz-cos(a).*dx, cos(a).*dx-sin(a).*dz, cos(a).*dx-sin(a).*dz; 
    -dy, dy,-dy, dy];

%t = solve(A,X);
%t= X/A;
t= inv(A)*X;
%fx=[sin(a).*t(1,1); -sin(a).*t(2,1); sin(a).*t(3,1); -sin(a).*t(4,1)];


%% LMN 계산

B = [ dy, -dy, -dy, dy; 
    sin(a).*dz-cos(a).*dx, sin(a).*dz-cos(a).*dx, cos(a).*dx-sin(a).*dz, cos(a).*dx-sin(a).*dz; 
    -dy, dy,-dy, dy];

m = B*t;

end