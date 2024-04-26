function J_s = jacobe_s(S,M,q) 
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
end
