%prepare the workspace
clear;
close all;

%run('init');
%addpath('kinematics/');

import utils.get_initial_configuration;

% inverse or transpose (of Jacobian)
algorithm = 'inverse';
% desired position
x_pos = [0.65 0 0.4]';
% desired quaternion
x_dir = [1 -1 0]';
x_dir = x_dir / norm(x_dir);
theta = pi/2;
x_q = [cos(0.5*theta); sin(0.5*theta)*x_dir];

q = get_initial_configuration(x_pos, x_q, algorithm);
