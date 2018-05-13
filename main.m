%prepare the workspace
clear;
close all;

run('init');
%addpath('kinematics/');
addpath('drawing/');

import utils.get_initial_configuration;
import utils.get_init_rot_matrix;
import utils.get_rot_matrix;
import utils.get_trajectory;
import utils.get_qvrep;
import utils.plot_rot_matrix;

import kinematics.get_quaternion;
import kinematics.DirectKinematics;

% inverse or transpose (of Jacobian)
algorithm = 'inverse';
% time of simulation, seconds
tf = 30;
% initial position
x_init = [0.65 0 0.4]';
% desired quaternion
x_dir = [1 -1 0]';
x_dir = x_dir / norm(x_dir);
% diplacement of the end effector
s = 0.05;
%s = 0; % no translation
x_f = s*x_dir + x_init;
%x_f = x_init;
theta = 4*pi;

r_i = get_init_rot_matrix(x_dir);
x_q_i = get_quaternion(r_i);
q_init = get_initial_configuration(x_init, x_q_i, algorithm, DH_kuka);

DH_kuka(:, 4) = q_init;
%K = diag([0.25 0.25 0.25]);
gain_pos = 0.25;
gain_angle = 500;
K = diag([gain_pos gain_pos gain_pos gain_angle gain_angle gain_angle]);
[t, q_t, dq_t] = get_trajectory(DH_kuka, x_f, tf, x_dir, theta, algorithm, K);

figure;
subplot(2, 1, 1);
plot(t, q_t);
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7');
title('Joint variables over time');

subplot(2, 1, 2);
plot(t, dq_t);
legend('dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7');
title('Joint variables velocities over time');


main_vrep(q_init, q_t);
%main_vrep(q_t(:, end));
%q_t_vrep = get_qvrep(q_t(:, i));