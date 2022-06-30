function angle_cmd = acc2angle(acc)
% acc = [ax ay ax]
global roll0

yaw = atan(acc(2)/acc(1));
pitch = -atan(norm([acc(2) acc(1)])/-acc(3));
%pitch = atan2(-acc(1),-acc(3));
if pitch > 0
    pitch = pitch + 2*pi;
end

roll = 0;

angle_cmd = [roll pitch yaw];

end