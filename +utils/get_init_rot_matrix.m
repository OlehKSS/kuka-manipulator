function rot_matrix = get_init_rot_matrix(axis)
%GET_INIT_ROT_MATRIX returns rotation matrix for the given axis, 
% rotation is performed so x will align with the provided axis.

% let's find orthogonal basis
x = axis/norm(axis);
y = [-axis(3) 0 axis(1)]';
z = cross(x, y);
y = y/norm(y);
z = z/norm(z);

% test
%disp(dot(x, cross(y, z)));

rot_matrix = [x y z];
end

