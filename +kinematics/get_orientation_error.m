function error = get_orientation_error(qe, qd)
% qe - first quaternion, actual
% qd - second quaternion, desired
% 
% if (qe(1) < 0) % I need to check only this
%     % since two quaternions [a, b] = [-a, -b] represent 
%     % I will need to add this type of check for every time I do
%     % quaternion calculation
%     % the same rotation
%     qe(1) = -qe(1);
%     qe(2:end) = -qe(2:end);    
% end
% 
% if (qd(1) < 0)
%     qd(1) = -qd(1);
%     qd(2:end) = -qd(2:end);
% end

error = qe(1)*qd(2:end) - qd(1)*qe(2:end) - cross(qd(2:end),qe(2:end));

scalar_error = qe(1)*qd(1) + (qd(2:end)')*qe(2:end);

    if (scalar_error < 0)
        error = -error;
    end
end