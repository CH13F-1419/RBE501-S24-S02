


%YOUR INVERSE KINEMATICS SOLVER GOES HERE
function q = ikin(S,M,q,V)
  
    % Joint limits
qlim = [-pi/2  pi/2;  % q(1)
        -pi/4  pi/2;  % q(2)
        -pi/12 pi/3]; % q(3)

    no_iterations = 10000;
    threshold_1 = 100;
    %Generate the robot's pose
    currentQ = q;
    T = fkine(S,M,q,'space');
    currentPose = T(1:3,4);
    targetPose = V;
    lambda = 2;
    iter = 0;
    flag = 0;
    while norm(targetPose - currentPose) > 1e-6 
        if iter > no_iterations
            flag = 1;
            break
        end 
        J = jacoba(S,M,currentQ); 
        deltaQ = J' * pinv(J*J' + lambda^2 * (eye(3))) * (targetPose - currentPose);
         %deltaQ = J' * (targetPose - currentPose);       
        
        currentQ = currentQ + deltaQ';
        %currentQ = min(max(qlim(:,1)', currentQ), qlim(:,2)');

        T = fkine(S,M,currentQ,'space');
        %currentPose = MatrixLog6(T);
        % currentPose = [currentPose(3,2) ...
        %                currentPose(1,3) ...
        %                currentPose(2,1) ...
        %                currentPose(1:3,4)']';
        currentPose = T(1:3, 4);
        
        iter = iter + 1;
    end            
q = currentQ; 

%q = currentPose;
%qlim(:,1)
%currentQ
%q = currentQ;
%disp(iter);
% = currentQ;
end 


