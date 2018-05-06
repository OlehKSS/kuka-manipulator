%prepare the workspace
clear;
close all;

run('init');
%addpath('kinematics/');

import utils.get_initial_configuration;
import utils.get_init_rot_matrix;
import utils.get_rot_matrix;
import utils.get_trajectory;

import kinematics.get_quaternion;
import kinematics.DirectKinematics;

% inverse or transpose (of Jacobian)
algorithm = 'inverse';
% time of simulation, seconds
tf = 20;
% initial position
x_init = [0.65 0 0.4]';
% desired quaternion
x_dir = [1 -1 0]';
x_dir = x_dir / norm(x_dir);
% diplacement of the end effector
s = 0.05;
x_f = s*x_dir + x_init;
%x_f = x_init;
theta = 4*pi;

r_i = get_init_rot_matrix(x_dir);
x_q_i = get_quaternion(r_i);
q_init = get_initial_configuration(x_init, x_q_i, algorithm, DH_kuka);

DH_kuka(:, 4) = q_init;
%K = diag([0.25 0.25 0.25]);
K = diag([0.25 0.25 0.25 0.1 0.1 0.1]);
q_t = get_trajectory(DH_kuka, x_f, tf, x_dir, theta, algorithm, K);

main_vrep(q_init, q_t);
