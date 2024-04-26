function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%
% Forward iterations
size_vector = size(params.jointPos);    % Number of links in the kinematic chain
n = size_vector(1);
g = params.g(3);     % [m/s2] Gravity acceleration (aligned with the Y axis)
M = params.M;
%YOUR CODE HERE 
M_space = zeros(4,4,n+1);
M_temp = eye(4,4);
for i = 1: n+1
    i;
    M_space(:,:,i) = M_temp * params.M(:,:,i);
    M_temp  = M_space(:,:,i);
end
%M01 = params.M(:,:,1);
%M12 = params.M(:,:,2);
%M23 = params.M(:,:,3);
%M34 = params.M(:,:,4)

%####
%M1 = M01;
%M2 = M1 * M12;
%M3 = M2 * M23;
%M4 = M3 * M34;

%% calulcate screw axis in space frame
A = zeros(6,n);
for i = 1: n
    A(:,i) = adjoint(pinv(M_space(:,:,i))) * params.S(:, i);
end
%disp("###################")
%disp("##################");
%###
%A(:, 1) = adjoint(pinv(M1)) * params.S(:, 1);
%A(:, 2) = adjoint(pinv(M2)) * params.S(:, 2);
%A(:, 3) = adjoint(pinv(M3)) * params.S(:, 3);



%%Initialize the twists and accelerations of each link
V = zeros(6,n+1);
Vd = zeros(6,n+1);
%%graviational acceleration
g = params.g;
Vd(:,1) = [0 0 0 -g(1) -g(2) -g(3)]';

%%forward propapgation 
for i = 1:n   
    if i == 1 
        T = twist2ht(-A(:,i), params.jointPos(i));
        T = T * params.M(:,:,i);
        T = T \ eye(4);
        adjT = adjoint(T);   
               
    else
        T = twist2ht(A(:,i), params.jointPos(i));
        T = params.M(:,:,i)* T;
        T = T \ eye(4);
        adjT = adjoint(T);
        %adjT = adjoint()
        
    end
    %adjT = adjoint(T_i(:,:,i));  
    
    V(:,i+1) =  A(:,i) * params.jointVel(i) + adjT * V(:,i); % Link Velocity
    Vd(:,i+1) = A(:,i) * params.jointAcc(i) +  adjT * Vd(:,i) +  ad(V(:, i+1)) * A(:,i)* params.jointVel(i); % Link Acceleration

end
% Backward iterations
F = zeros(6,n);
tau = zeros(1,n)';
%disp("here....")
for i = n:-1:1    
    if i == n 
       %T = twist2ht(-A(:,i), params.jointPos(i));
       T = params.M(:,:,i+1);
       T = T \ eye(4);
       adjT = adjoint(T);  
       F(:,i) = (params.G(:,:,i) * Vd(:,i+1)) - ad(V(:,i+1))' * params.G(:,:,i) * V(:,i+1)   +  ( adjT )' * params.Ftip;
       tau(i) = F(:,i)' * A(:,i);
    else
       T = twist2ht(A(:,i+1), params.jointPos(i+1));
       T = params.M(:,:,i+1)* T;
       T = T \ eye(4);
       adjT = adjoint(T);
       F(:,i) = (params.G(:,:,i) * Vd(:,i+1)) - ad(V(:,i+1))' * params.G(:,:,i) * V(:,i+1) +  ( adjT )' * F(:,i+1);
       tau(i) = F(:,i)' * A(:,i);
       %dips(tau(i))
       
    end

    end


%Vdot = cat(2,Vd0, Vd);
%V = cat(2,V0, V)
Vdot = Vd;
tau(:,1);
end

%%other functions

function AdT = adjoint(T)
    % your code here
    R = T(1:3, 1:3);
    p = T(1:3,4);
    p_mat = [ 0, -p(3), p(2);
                p(3), 0, -p(1);
               -p(2), p(1), 0 ];
    
    AdT = [R, zeros(3);
                p_mat*R , R ];
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

function R = axisangle2rot(omega,theta)
    % your code here
    
    omega_mat = skew_mat(omega);
    R = eye(3,3) + sin(theta) * omega_mat + (1 - cos(theta)) * omega_mat^2 ;      
end

function omega_mat = skew_mat(omega)
omega_mat = [ 0,  -omega(3), omega(2);
                omega(3), 0, -omega(1);
                -omega(2), omega(1), 0 ];

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