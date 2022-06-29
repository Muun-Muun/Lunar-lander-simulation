function ang_new = ang_limit(ang)
% ang limitation
% : 0 <= ang < 2*pi
for i = 1 : 3
    if ang(i) >= pi
        while ang(i) > pi
            ang(i) = ang(i) - 2*pi;
        end

    elseif ang(i) < -pi
        while ang(i) <= -pi
            ang(i) = ang(i) + 2*pi;
        end
    end

    ang_new = ang;
end

end