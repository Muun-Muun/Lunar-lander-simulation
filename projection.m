function temp_x = projection(x)    
global n lb ub
temp_x = zeros(3,n);
x = reshape(x, [3,n]);
for i = 1: n
    index = x(:,i);
    if norm(index) < lb || norm(index) > ub

        temp = zeros(3,1);
        for j = 1: 3
            index2 = index(j);
            temp(j) = index2/norm(index)*ub;
        end
        temp_x(:,i) = temp;
    else
        temp_x(:,i) = index;
    end

end
temp_x = reshape(temp_x, [n*3,1]);
end
        