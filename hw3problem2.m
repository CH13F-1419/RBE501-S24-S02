% RBE 501 - Robot Dynamics - Spring 2024
% Homework 3, Problem 2
% Worcester Polytechnic Institute
%% Instructor: L. Fichera <loris@wpi.edu>
% Last modified: 03/10/2024
clear, clc, close all
nTests = 20;
%%Create the manipulator
mdl_stanford
stanf
plotOn = true;
% Display the manipulator
stanf.teach(zeros(1,6)), hold on;
% Load the path that we wish to follow and display it
load hw3problem2_path.mat
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
%% YOUR CODE HERE
S_body = [0 0 1 0 -0.154 0;  -1 0 0 0 0.263 0;  0 0 0 0 0 1;  0 0 1 0 0 0;  0 1 0 0.263 0 0;  0 0 1 0 0 0]'; 
M = [0 -1 0 0; 1 0 0 0 ; 0 0 1 0 ; 0 0.154 0.675 1 ]';
% Joint limits
% Joint limits
qlim = [-2.9671*180/pi  2.9671*180/pi;  % q(1)
        -2.9671*180/pi  2.9671*180/pi;  % q(2)
         0.3048   1.27;
        -2.9671*180/pi  2.9671*180/pi;
        -1.5708*180/pi  1.5708*180/pi;
        -2.9671*180/pi  2.9671*180/pi];

%% Part B - Calculate the forward kinematics with the Product of Exponentials formula in the body frame
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration
fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
 
% Test the forward kinematics for 20 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
     
    % Calculate the forward kinematics
    T = fkine(S_body,M,q,'body');
    
    if plotOn
        stanf.teach(q);
        title('Forward Kinematics Test');
    end
    err = abs(double(stanf.fkine(q)) - T);
    assert(all(all(abs(double(stanf.fkine(q)) - T) < 1e-10)));
end 
fprintf('\nTest passed successfully.\n');


%% Part C - Calculate the Body Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

S_space = [0 0 1 0 0 0; 
    0 1 0 -0.412 0 0; 
    0 0 0 0 0 1; 
    0 0 1 0.154 0 0; 
    1 0 0 0 0.412 -0.154; 
    0 0 1 0.154 0 0]';

% Test the correctness of the Jacobian for 20 random sets of joint
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));

      % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];

    
   % Calculate the Jacobian in the body frame
    J_b = jacobe(S_space,M,q) ;

    if plotOn
        stanf.teach(q);
        title('Differential Kinematics Test');
    end

    % Test the correctness of the Jacobian
    J_test = [J_b(4:6,:); J_b(1:3,:)]; % swap the rotation and translation components 
    assert(all(all(abs(double(stanf.jacobe(q)) - J_test) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


%% Part D - Inverse Kinematics
% Load the path that we wish the robot to follow and display it
load hw3problem2_path.mat
path_dim = size(path);
%path_dim = path_dim(2);
s_size = size( S_body);
q_no = s_size(1);

stanf.plot(zeros(1, q_no)), hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

q_value = zeros(path_dim(2), q_no);
for i = 1 : path_dim(2) 
    q0 = zeros(1,q_no);
    q = ikin(S_space,M,q0,path(:,i));
    q_value(i,:) = q;    
end
if plotOn
        stanf.teach(q_value);
        title('Inverse Kinematics Test');
end


