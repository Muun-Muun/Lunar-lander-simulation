function ret = confirm_bds(x)
    global lb ub n
    x = reshape(x, [3,n]);
    for i = 1 : n
      if norm(x(:,i)) < lb || norm(x(:,i)) > ub
        ret =  false;
        return
      end
    end
    ret = true;
end
    

