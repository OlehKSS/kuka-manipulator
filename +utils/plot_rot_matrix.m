function plot_rot_matrix(R, color)
    % Plots rotation matrix R, color is hue of the vectors 
    if (nargin < 2)
        color = [0 0 1];
    end
    
    basis_x = R(1:3, 1);
    basis_y = R(1:3, 2);
    basis_z = R(1:3, 3);
    quiver3(0, 0, 0, basis_x(1), basis_x(2), basis_x(3), 'color', color);
    hold on;
    quiver3(0, 0, 0, basis_y(1), basis_y(2), basis_y(3), 'color', color);
    hold on;
    quiver3(0, 0, 0, basis_z(1), basis_z(2), basis_z(3), 'color', color);
end

