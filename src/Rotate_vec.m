function vec2 = Rotate_vec(vec1, ang, type)
% vec: pos vel ... (column vector)
% ang: angle between coordinations, ang = [roll pitch yaw]
% type: 'n2b' or 'b2n', str type
   
C_n2b = [1 0 0; 0 cos(ang(1)) sin(ang(1)); 0 -sin(ang(1)) cos(ang(1))]...
    *[cos(ang(2)) 0 -sin(ang(2)); 0 1 0; sin(ang(2)) 0 cos(ang(2))]...
    *[cos(ang(3)) sin(ang(3)) 0; -sin(ang(3)) cos(ang(3)) 0; 0 0 1];   % 3-2-1 rotation

C_I2n = [1 0 0;0 cos(ang(1)) sin(ang(1));0 -sin(ang(1)) cos(ang(1))]...
    *[cos(ang(3)) sin(ang(3)) 0;-sin(ang(3)) cos(ang(3)) 0;0 0 1];     % 3-1 rotation

switch type
    case 'n2b'
        vec2 = C_n2b*vec1;
    case 'b2n'
        vec2 = C_n2b'*vec1;
    case 'I2n'
        vec2 = C_I2n*vec1;
    case 'n2I'
        vec2 = C_I2n'*vec1;
end

end