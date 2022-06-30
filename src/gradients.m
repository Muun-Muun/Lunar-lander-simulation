function gradients(start_point)
    global point k_max k2_max 

    i = 0;
    point = start_point; 
    if confirm_bds(point) == false
      point = projection(point);
    end

    while(i < k_max)
      j = 0;
      while(j < k2_max)
        min_obj = minimize(point);
        switch min_obj
            case true
                break
            case 'break'
                break
        end
        j = j + 1;
      end
      
      switch min_obj
          case 'break'
            break 
      end
      i = i + 1;
    end
end
