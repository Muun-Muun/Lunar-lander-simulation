function ret = minimize(x)
global point lr A_tilde b_tilde epsilon G Q first
grad_temp = 2*A_tilde'*(A_tilde*x-b_tilde);
temp = x - grad_temp*lr;
standard = 3;

if confirm_bds(temp) == false 
  temp = projection(temp);
end

if norm(A_tilde*temp-b_tilde) > norm(A_tilde*x-b_tilde)
  lr = lr/2;
  ret = false;
  return
else
  if norm(temp - point) < epsilon
      temp_state = G*point + Q +first;
      for i = 1: length(temp_state)
          if abs(temp_state(i)) > standard
              lr = 1.2*lr;
              point = temp;
              ret = true;
          end
      end
      ret = 'break';
      return
  else
      lr = 1.2*lr;
      point = temp;
      ret = true;
  end
end
end

