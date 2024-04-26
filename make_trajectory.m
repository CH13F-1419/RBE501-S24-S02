function traj = make_trajectory(type, params)
% YOUR CODE HERE
trajectory.t = params.t(1):params.dt:params.t(2);
if strcmp(type, 'cubic') 
    %% degree 4 
    coefficients = zeros(4,1);
    q_Vector = [params.q(1) params.v(1) params.q(2) params.v(2)]';
    t_Matrix = [1 params.t(1) params.t(1)^2 params.t(1)^3;
                    0 1 2*params.t(1) 3*params.t(1)^2;
                    1 params.t(2) params.t(2)^2 params.t(2)^3;
                    0 1 2*params.t(2) 3*params.t(2)^2];
     coefficients = t_Matrix \ q_Vector;
     a0 = coefficients(1);
     a1 = coefficients(2);
     a2 = coefficients(3);
     a3 = coefficients(4);

     traj.q = a0 + a1 * trajectory.t + a2 * trajectory.t.^2 + a3 * trajectory.t.^3;
     traj.v = a1 + 2 * a2 * trajectory.t + 3 * a3 * trajectory.t.^2;
     traj.a = 2 * a2 + 6 * a3 * trajectory.t;
     traj.t = trajectory.t;

elseif strcmp(type, 'quintic')
      %% this is function with degree 5 
        coefficients = zeros(6,1);
        qVector = [params.q(1) params.v(1) params.a(1) params.q(2) params.v(2) params.a(2)]';
        tMatrix = [1 params.t(1) params.t(1)^2 params.t(1)^3 params.t(1)^4 params.t(1)^5;
                    0 1 2*params.t(1) 3*params.t(1)^2 4*params.t(1)^3 5*params.t(1)^4;
                    0 0 2 6*params.t(1) 12*params.t(1)^2 20*params.t(1)^3;
                    1 params.t(2) params.t(2)^2 params.t(2)^3 params.t(2)^4 params.t(2)^5;
                    0 1 2*params.t(2) 3*params.t(2)^2 4*params.t(2)^3 5*params.t(2)^4;
                    0 0 2 6*params.t(2) 12*params.t(2)^2 20*params.t(2)^3];
        coefficients = tMatrix \ qVector;
        a0 = coefficients(1);
        a1 = coefficients(2);
        a2 = coefficients(3);
        a3 = coefficients(4);
        a4 = coefficients(5);
        a5 = coefficients(6);

        traj.q = a0 + a1 * trajectory.t + a2 * trajectory.t.^2 + a3 * trajectory.t.^3 + a4 * trajectory.t.^4 + a5 * trajectory.t.^5;
        traj.v = a1 + 2 * a2 * trajectory.t + 3 * a3 * trajectory.t.^2 + 4 * a4 * trajectory.t.^3 + 5 * a5 * trajectory.t.^4;
        traj.a = 2 * a2 + 6 * a3 * trajectory.t + 12 * a4 * trajectory.t.^2 + 20 * a5 * trajectory.t.^3;
        traj.t = trajectory.t;
    else
        error('Invalid Input');
    end


end



