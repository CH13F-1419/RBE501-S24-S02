function adV = ad(V)
    % your code here
    w = V(1:3);
    v = V(4:6);
    w_mat = skew_mat(w);
    v_mat = skew_mat(v);
    adV = [w_mat , zeros(3,3);
           v_mat, w_mat];
end

function p_mat = skew_mat(p)    
    p_mat = [ 0, -p(3), p(2);
                p(3), 0, -p(1);
               -p(2), p(1), 0 ];
end
