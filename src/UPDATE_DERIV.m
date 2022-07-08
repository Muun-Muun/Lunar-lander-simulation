function [X,dX] = UPDATE_DERIV(Moment, acc, state)
global MOI
% format:
%   X (or state) = [position velocity euler_angle angualr_velocity]
%   dX = [d(position)/dt d(velocity)/dt d(euler_angle)/dt d(angular_velocity)/dt]

% get states
vel = state(4:6);       % velocity  
ang = state(7:9);       % euler angle
ang_vel = state(10:12); % angular velocity

% calculate derives of states
dpos = cal_derive_position(ang,vel);
dvel = cal_derive_velocity(vel,ang,ang_vel,acc);
dang = cal_derive_euler_angle(ang,ang_vel);
dang_vel = cal_derive_angular_velocity(Moment,MOI,ang_vel);

% states and derives of states
X = state;
dX = [dpos' dvel' dang' dang_vel'];

%% Sub Functions
    % claculate derive of position
    function dpos = cal_derive_position(ang,vel)
        dpos = DCM_b2n(ang)*vel';
    end
    % calculate derive of Euler angle
    function dang = cal_derive_euler_angle(ang,angular_velocity)
        dang = [1 sin(ang(1))*tan(ang(2)) cos(ang(1))*tan(ang(2));...
            0 cos(ang(1)) -sin(ang(1));...
            0 sin(ang(1))*sec(ang(2)) cos(ang(1))*sec(ang(2))]*angular_velocity';
    end

    % claculate derive of velocity
    function dvel = cal_derive_velocity(vel,ang,ang_vel,acc)
        Thrust = [0 0 -norm(acc)]';  
        Gravity = DCM_b2n(ang)'*[0 0 1.63]';
        dvel = -cross(ang_vel,vel)' + Thrust + Gravity;
    end

    % claculate derive of velocity
    function dang_vel = cal_derive_angular_velocity(Moment,MOI,ang_vel)
        ang_moment = MOI*ang_vel';
        dang_vel = MOI\(Moment' - cross(ang_vel,ang_moment)');
    end

end

