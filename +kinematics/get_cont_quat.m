function quat_current = get_cont_quat(R, quat_t, i)
    % Function keeps quaternions continous
    % R - rotation matrix
    % quat_t - previously obtained quaternions
    % i - index of iteration (moment of time), for which we are finding
    % continuous quaternion
    import kinematics.get_quaternion;

    quat_current = get_quaternion(R);
    % get the maximum value of quaternion elements
    [qc_max, qc_index] = max(quat_current);

    % if signs of current quaternion and previous quaternion are different
    % then use -q instead
    if i > 1 && qc_max*quat_t(qc_index, i-1)<0
        quat_current = -quat_current;
    end
end

