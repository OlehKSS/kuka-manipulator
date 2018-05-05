function rot_matrix = get_rot_matrix(axis, theta)
%GET_ROT_MATRIX returns rotation matrix for the given axis and angle
% axis - vector rotation should be done about
% theta - angle of rotation
    x = axis/norm(axis);
    % cross product matrix for the x
    cross_m = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    rot_matrix = cos(theta)*eye(3,3) + sin(theta)*cross_m + (1 - cos(theta))*kron(x, x');
end
