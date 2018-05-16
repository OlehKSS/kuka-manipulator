function q = get_quaternion(R)
    % Returns quaternion for the given rotation matrix
    % R - rotation matrix to be transformed
    eta = 0.5 * sqrt(R(1,1) + R(2,2) + R(3,3) +1);
    
    if (R(3,2) - R(2,3) >= 0)
        e1 = 0.5*sqrt(R(1,1) - R(2,2) - R(3,3) + 1);
    else
        e1 = -0.5*sqrt(R(1,1) - R(2,2) - R(3,3) + 1);
    end
    
    if (R(1,3) - R(3,1) >= 0)
        e2 = 0.5*sqrt(R(2,2) - R(3,3) - R(1,1) + 1);
    else
        e2 = -0.5*sqrt(R(2,2) - R(3,3) - R(1,1) + 1);
    end
    
    if (R(2,1) - R(1,2) >= 0)
        e3 = 0.5*sqrt(R(3,3) - R(1,1) - R(2,2) + 1);
    else
        e3 = -0.5*sqrt(R(3,3) - R(1,1) - R(2,2) + 1);
    end
    
    epsilon = [e1; e2; e3];

    % since two quaternions [a, b] = [-a, -b] represent the same rotation, we
    % need to stick to one representation only, we need to check only scalar
    % part of the quaternion
%     if (eta < 0)
%         eta = -eta;
%         epsilon = -epsilon;
%     end
    
    q = [eta; epsilon];
    q = real(q);
    q = q/norm(q);
end