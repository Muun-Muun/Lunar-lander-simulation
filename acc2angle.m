function angle_cmd = acc2angle(acc)
% acc = [ax ay ax]
global roll0

yaw = atan2(acc(2),acc(1));
pitch = atan2(-1*acc(3),norm(acc(1:2)));
roll = roll0;

angle_cmd = [yaw pitch roll];

end