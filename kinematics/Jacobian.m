function J = Jacobian(T)
% Computes geometric Jacobian for a manipulator made of revolute joints
% T - cell array of direct kinematic functions

    number_of_joints = length(T);
    % final direct kinematics function
    dir_kin = T{number_of_joints};
    
    p0 = [0; 0; 0; 1];
    z0 = [0; 0; 1];
    pe = dir_kin * p0;
    pe = pe(1:3);

    J = zeros(6, number_of_joints);
    
    J(1:3, 1) = cross(z0, (pe - p0(1:3)));
    J(4:6, 1) = z0;

    for i = 1:(number_of_joints - 1)
        temp_Ti = T{i};
        temp_pi = temp_Ti*p0;
        temp_pi = temp_pi(1:3);
        temp_zi = temp_Ti(1:3, 1:3)*z0;
        J(1:3, i+1) = cross(temp_zi, (pe - temp_pi));
        J(4:6, i+1) = temp_zi;    
    end
end