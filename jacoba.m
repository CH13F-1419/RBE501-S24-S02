function J_a = jacoba(S,M,q)    
    % your code here
    J_s = jacobe_s(S,M,q);
    T = fkine(S,M,q,'space');
    p = T(1:3, 4);
    J_v  = J_s(4:6,:);
    J_w = J_s(1:3,:);
    p = [ 0,  -p(3), p(2);
          p(3), 0, -p(1);
         -p(2), p(1), 0 ];      
    J_a = (J_v - p*J_w);
end


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
    J_s = J;
end

function Vtrans = adjoints(V,T)
    % your code here
    R = T(1:3, 1:3);
    p = T(1:3,4);
    p_mat = [ 0, -p(3), p(2);
                p(3), 0, -p(1);
               -p(2), p(1), 0 ];
    
    adjoint = [R, zeros(3);
                p_mat*R , R ];
    Vtrans = adjoint * V;
    
    
end

function T = fkine(S,M,q,frame)
   
    dim = size(q);
    T = eye(4);
    %PoE formulae for Transofrmation calculations
    for i=1:dim(2)
        T = T*twist2ht(S(:,i),q(i));
    end
    if (frame == "body")
        T = M * T;
    else
        T = T * M;
    end
end

function T = twist2ht(S,theta)
    % your code here
    v = S(4:6);
    omega = S(1:3);
    % If needed, you can calculate a rotation matrix with:
    R = axisangle2rot(omega,theta);
    omega_mat = [ 0,  -omega(3), omega(2);
                omega(3), 0, -omega(1);
                -omega(2), omega(1), 0 ];
    
    T = [R , ( eye(3) * theta + (1-cos(theta))* omega_mat + (theta - sin(theta)) * omega_mat^2)* v;
        0, 0,0,  1];
end
