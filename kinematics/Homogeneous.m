function A = Homogeneous(dh_row)
%dh_row is a row of Denavit-Hartenberg parameters of a joint
%returns a homogeneous transformation matrix
    a = dh_row(1);
    alpha = dh_row(2);
    d = dh_row(3);
    theta = dh_row(4);

    A = zeros(4, 4);

    A(1, 1) = cos(theta);
    A(1, 2) = -sin(theta)*cos(alpha);
    A(1, 3) = sin(theta)*sin(alpha);
    A(1, 4) = a*cos(theta);

    A(2, 1) = sin(theta);
    A(2, 2) = cos(theta)*cos(alpha);
    A(2, 3) = -cos(theta)*sin(alpha);
    A(2, 4) = a*sin(theta);

    A(3, 2) = sin(alpha);
    A(3, 3) = cos(alpha);
    A(3, 4) = d;

    A(4, 4) = 1;
end