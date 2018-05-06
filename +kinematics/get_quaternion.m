function q = get_quaternion(R)
% R - rotation matrix to be transformed
etha = 0.5 * sqrt(R(1,1) + R(2,2) + R(3,3) +1);

epsilon = [
    sign(R(3,2) - R(2,3))*0.5*sqrt(R(1,1) - R(2,2) - R(3,3) + 1);
    sign(R(1,3) - R(3,1))*0.5*sqrt(R(2,2) - R(3,3) - R(1,1) + 1);
    sign(R(2,1) - R(1,2))*0.5*sqrt(R(3,3) - R(1,1) - R(2,2) + 1);
    ];

q = [etha; epsilon];
q = real(q);
q = q/norm(q);
end