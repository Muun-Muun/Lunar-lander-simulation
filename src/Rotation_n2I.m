function C_n2I = Rotation_n2I(beta)

C_n2I = [cos(beta) sin(beta) 0; -sin(beta) cos(beta) 0; 0 0 1]...
    *[cos(pi) 0 -sin(pi); 0 1 0; sin(pi) 0 cos(pi)];

end