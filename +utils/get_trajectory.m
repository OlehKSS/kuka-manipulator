function [t, q_t, dq_t] = get_trajectory(DH, x_f, tf, axis, theta, algorithm, K)
    %GET_TRAJECTORY Summary of this function goes here
    %   Detailed explanation goes here
    % GET_INITIAL_CONFIGURATION returns initail joint variables for the
    % specified end effector position x_f and quaternion x_q
    % algorithm is string that defines a method for inverse differential 
    % kinematics calculation, equals to 'inverse' or 'transpose'
    % DH - Denavit-Hartenberg table for the Kuka, initial configuration
    % K - gain matrix, optional
    % x_f - final position of the end effector
    % axis - rotation axis
    % theta - value of rotation angle about the given axis
    % tf - time of simulation
    
    import kinematics.get_orientation_error;
    import kinematics.get_quaternion;
    import kinematics.get_cont_quat;
    import kinematics.trapezoidal;
    import kinematics.Jacobian;
    import kinematics.DirectKinematics;
    
    import utils.get_rot_matrix;
    import utils.get_rot_matrix;
    import utils.get_init_rot_matrix;
    import utils.plot_rot_matrix;
    
    % sampling time
    dt = 0.001;
    t = 0:dt:tf;
    t_trapzl = 0.7*tf;
    N = length(t);
    % number of links
    n = size(DH, 1);

    q_t = zeros(n, N);
    q_vrep = zeros(n, N);
    q_t(:, 1) = DH(:, 4);
    dq_t = zeros(n, N);
    quat_t = zeros(4, N);
    
    pos_t = zeros(3, N);
    pos_error_t = zeros(3, N);
    quat_error_t = zeros(3, N);
    
    axis = axis/norm(axis);
    theta_t = zeros(size(t));
    theta_vt = zeros(size(t));
    theta_at = zeros(size(t));
    
    if (nargin < 7)
        % the larger K the larger robot movement
        % settling time will be 10*constant time (inverse of K diagonal values)
        K = diag([100 100 100 80 80 80]);
    end
    

    if strcmp(algorithm, 'inverse')
        % do I need to use velocity here as well e*(1/dt) 
        dq = @(e, J_a) (pinv(J_a)*(K*e));    
    else
        dq = @(e, J_a) (J_a'*(K*e)); 
    end
    
    % I should decouple rotation and translation. Translation should be
    % done in the same way (with differential kinematics), but in case of
    % rotation I should make a trapezoidal velocity profile and use
    % expresion for matrix multiplication.
    
    % lets find velocity profile for the whole silmulation time
    % cruise velocity, 1 < coef <= 2
    theta_vc = 1.4*theta/t_trapzl;
    
    for i = 1:N
        [theta_t(i), theta_vt(i), theta_at(i)] = trapezoidal(0, theta, theta_vc, t_trapzl, t(i));
    end
    
    % init quaternion
    T = DirectKinematics(DH);
    T = T{end};
    r_i = T(1:3, 1:3);
    plot_rot_matrix(r_i);
    hold on;
    quiver3(0, 0, 0, 1, -1, 0, 'color', [0 1 0]);
    hold on;
    
    % function for obtaining rotation matrix for each time instant
    %R_t = @(i) r_i*get_rot_matrix(axis, theta_t(i));
    R_t = @(i) r_i*rotz(rad2deg(theta_t(i)));
    %R_t = @(i) [1 0 0; 0 cos(theta_t(i)) sin(theta_t(i)); 0 -sin(theta_t(i)) cos(theta_t(i))];

%figure;
%DrawRobot(DH);

    for i = 1:(N - 1)
        kuka_joint_temp = q_t(:, i);
        DH(:, 4) = kuka_joint_temp;
        
        T = DirectKinematics(DH);
        J = Jacobian(T);
        T = T{end};
        % In case of quaternion representation geometric Jacobian and
        % analytical are the same 

        x_f_current = T(1:3, 4);       
        x_q_current = get_quaternion(T(1:3, 1:3));
        %x_q_current = get_quaternion(R_t(i));
        x_q = get_cont_quat(R_t(i), quat_t, i);
        %x_q = [1, 0, 0, 0]';
        quat_t(:, i) = x_q;
        if (mod(i, 500) == 0) && (i <=3000)
            plot_rot_matrix(R_t(i), [1 0 0]);
            hold on;
        end
        % calculate separately error for position
        pos_t(:, i) = x_f_current;
        error_pos = (x_f - x_f_current);
        pos_error_t(:, i) = error_pos;
        error_quat = get_orientation_error(x_q_current, x_q);
        quat_error_t(:, i) = error_quat;
        error = [error_pos; error_quat];
        
        %dq_t(:, i+1) = dq(error_pos, J(1:3, :));
        dq_t(:, i) = dq(error, J);
        q_t(:, i + 1) = q_t(:, i) + dt * dq_t(:, i);
    end
    
    %figure;
    %DrawRobot(DH);
    
    %plot_rot_matrix(R_t(N-1), [1 0 0]);
    
    figure;
    subplot(2, 1, 1);
    plot(t, pos_error_t);
    title('Position error');

    subplot(2, 1, 2);
    plot(t, quat_error_t);
    title('Quaternion error');
    
    figure;
    subplot(2, 1, 1);
    plot(t, pos_t)
    title('Position time function');
    
    subplot(2, 1, 2);
    plot(t, quat_t);
    title('Quatertion over time');
    legend('\eta', '\epsilon_x', '\epsilon_y', '\epsilon_z');
    
    figure;
    subplot(2, 1, 1);
    plot(t, theta_t);
    title('\theta');
    
    subplot(2, 1, 2);
    plot(t, theta_vt);
    title('\theta velocity');
    
end
