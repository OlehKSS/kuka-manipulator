%prepare the workspace
clear;
close all;

%run('init');
%addpath('kinematics/');
import kuka.kuka_directkinematics;
import kuka.kuka_J;

import kinematics.get_orientation_error;
import kinematics.get_quaternion;

% inverse or transpose (of Jacobian)
algorithm = 'inverse';
% sampling time
dt = 0.001;
t = 0:dt:1;
N = length(t);
% number of links
n = 7;%size(DH, 1);

kuka_joint_init = [0;
    0;
    0;
    0;
    0;
    0;
    0];

q_t = zeros(n, N);
q_t(:, 1) = kuka_joint_init;%DH(:, 4);
dq_t = zeros(n, N);
%x_t = zeros(n, N);
%x_d = [0.8 0.8 pi]';
% desired position
x_pos = [0.3 0.3 0.6]';
pos_error_t = zeros(3, N);
% desired quaternion
x_q = [0 0 0 1]';
quat_error_t = zeros(3, N);

if strcmp(algorithm, 'inverse')
    K = diag(20*ones(1, 6));
    % the larger K the larger robot movement
    % settling time will be 10*constant time (inverse of K diagonal values)
    dq = @(e, J_a) (pinv(J_a)*(K*e));    
else
    K = diag(7*ones(1, 6));
    dq = @(e, J_a) (J_a'*(K*e)); 
end

for i = 1:(N - 1)

    kuka_joint_temp = q_t(:, i);
    T = kuka_directkinematics(kuka_joint_temp);
    J = kuka_J(kuka_joint_temp); 
    % In case of quaternion representation geometric Jacobian and
    % analytical are the same 
    
    x_pos_current = T(1:3, 4);       
    x_q_current = get_quaternion(T(1:3, 1:3));
    % calculate separately error for the quaternion and pos vector
    error_pos = x_pos - x_pos_current;
    pos_error_t(:, i) = error_pos;
    error_quat = get_orientation_error(x_q, x_q_current);
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