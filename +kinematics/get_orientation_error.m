function error = get_orientation_error(qe, qd)
% qe - first quaternion, actual
% qd - second quaternion, desired

error = qe(1)*qd(2:end) - qd(1)*qe(2:end) - cross(qd(2:end),qe(2:end));
end