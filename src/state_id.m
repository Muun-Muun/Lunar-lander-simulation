function id = state_id(n)
    switch n
        case 1
            id = 'x';
        case 2
            id = 'y';
        case 3
            id = 'z';
        case 4
            id = 'U';
        case 5
            id = 'V';
        case 6
            id = 'W';
        case 7
            id = 'roll';
        case 8
            id = 'pitch';
        case 9
            id = 'yaw';
        case 10
            id = 'P';
        case 11
            id = 'Q';
        case 12
            id = 'R';
    end
end