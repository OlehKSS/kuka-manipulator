function out = get_initial_configuration(x_pos, x_q, algorithm, DH, K)
    % GET_INITIAL_CONFIGURATION returns initail joint variables for the
    % specified end effector position x_pos and quaternion x_q
    % algorithm is string that defines a method for inverse differential 
    % kinematics calculation, equals to 'inverse' or 'transpose'
    % DH - Denavit-Hartenberg table for the Kuka
    % K - gain matrix, optional

    import kuka.kuka_directkinematics;
    import kuka.kuka_J;

    import kinematics.get_orientation_error;
    import kinematics.get_quaternion;
    import kinematics.Jacobian;
    import kinematics.DirectKinematics;

    % sampling time
    dt = 0.001;
    t = 0:dt:1;
    N = length(t);
    % number of links
    n = size(DH, 1);

    q_t = zeros(n, N);
    %q_t(:, 1) = kuka_joint_init;
    q_t(:, 1) = DH(:, 4);
    dq_t = zeros(n, N);

    pos_error_t = zeros(3, N);
    quat_error_t = zeros(3, N);
    
    if (nargin < 5)
        % the larger K the larger robot movement
        % settling time will be 10*constant time (inverse of K diagonal values)
        K = diag([100 100 100 80 80 80]);
    end
    

    if strcmp(algorithm, 'inverse')
        dq = @(e, J_a) (pinv(J_a)*(K*e));    
    else
        dq = @(e, J_a) (J_a'*(K*e)); 
    end

    for i = 1:(N - 1)

        kuka_joint_temp = q_t(:, i);
        DH(:, 4) = kuka_joint_temp;
        %T = kuka_directkinematics(kuka_joint_temp);
        %J = kuka_J(kuka_joint_temp); 
        T = DirectKinematics(DH);
        J = Jacobian(T);
        T = T{end};
        % In case of quaternion representation geometric Jacobian and
        % analytical are the same 

        x_pos_current = T(1:3, 4);       
        x_q_current = get_quaternion(T(1:3, 1:3));
        % calculate separately error for the quaternion and pos vector
        error_pos = x_pos - x_pos_current;
        pos_error_t(:, i) = error_pos;
        error_quat = get_orientation_error(x_q_current, x_q);
        quat_error_t(:, i) = error_quat;
        error = [error_pos; error_quat];
        dq_t(:, i+1) = dq(error, J);
        q_t(:, i + 1) = q_t(:, i) + dt * dq_t(:, i+1);

    end
    
    figure;
    plot(t, pos_error_t);
    title('Position error');

    figure;
    plot(t, quat_error_t);
    title('Quaternion error');
    
    out = q_t(:, end);
end

