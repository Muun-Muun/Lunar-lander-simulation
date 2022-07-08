function C_b2n = DCM_b2n(ang)
% calculate [ b-frame -> n-frame DCM ] of received angle
% Input: ang - angle used in DCM
% Output: C_b2n - converted vector

% calculate DCM (3-2-1 rotation)
C_b2n = [cos(ang(2))*cos(ang(3)) cos(ang(2))*sin(ang(3)) -sin(ang(2));...
    sin(ang(1))*sin(ang(2))-cos(ang(1))*sin(ang(3)) sin(ang(1))*sin(ang(2))*sin(ang(3))+cos(ang(1))*cos(ang(3)) sin(ang(1))*cos(ang(2));...
    cos(ang(1))*sin(ang(2))*cos(ang(3))+sin(ang(1))*sin(ang(3)) cos(ang(1))*sin(ang(2))*sin(ang(3))-sin(ang(1))*cos(ang(3)) cos(ang(1))*cos(ang(2))]';   

end