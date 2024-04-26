function J_b = jacobe(S,M,q)    
    % your code here
    dim = size(S);
    n = dim(2);
    T_matrix_list = cell(1, n);     
    J = S(:,1);
    T1 =  twist2ht(S(:,1),q(1));
    product = T1;
    for i = 2:n
        T_mat = twist2ht(S(:,i),q(i));        
        J_i = adjoints(S(:,i), product);
        product = product * T_mat;
        J = [J J_i];
    end   
    Js = J;
    %%
    T = fkine(S,M,q,'space');
    R = T(1:3, 1:3);
    p = T(1:3,4);
    p_mat = [ 0, -p(3), p(2);
                p(3), 0, -p(1);
               -p(2), p(1), 0 ];    
    adjoint = [R, zeros(3);
                p_mat*R , R ];
    J_b = pinv(adjoint)* Js;
    
end

