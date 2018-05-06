%prepare the workspace
clear;
close all;

run('init');
%addpath('kinematics/');

import utils.get_initial_configuration;
import utils.get_init_rot_matrix;
import utils.get_rot_matrix;
import utils.get_qvrep;
import kinematics.get_quaternion;
import kinematics.DirectKinematics;

% inverse or transpose (of Jacobian)
algorithm = 'inverse';
% initial position
x_pos = [0.65 0 0.4]';
% desired quaternion
x_dir = [1 -1 0]';
x_dir = x_dir / norm(x_dir);
theta = 4*pi;
x_q = [cos(0.5*theta); sin(0.5*theta)*x_dir];

r_i = get_init_rot_matrix(x_dir);
r_f = get_rot_matrix(x_dir, theta);
x_q_i = get_quaternion(r_i);
a = 1/sqrt(2);
R = [a 0 a; a 0 -a; 0 1 0];
quat_d0 = get_quaternion(R);
K = diag(10*ones(1, 6));
q = get_initial_configuration(x_pos, quat_d0, algorithm, DH_kuka, K);
q_vrep = get_qvrep(q);
main_vrep(q_vrep);
% how to find cruise velocity
% NOTE: A.*B multiplies A and B element by element. It is different from 
% A*B that is the matrix product of A and B
% dq_c = [1.5 1.6 1.8]'.*abs(q_f-q_i)/tf;