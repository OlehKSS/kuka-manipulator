%
%
% Template to visualize in V-REP the inverse kinematics algorithm developed
%          for a 7-DOF robot of the family Kuka LWR or IIWA
%
% Read Instructions.odt first !
% 
% Do not modify any part of this file except the strings within
%    the symbols << >>
%
% G. Antonelli, Sistemi Robotici, fall 2014

function [t, q, q_act] = main_vrep

    porta = 19997;          % default V-REP port
    tf = 4;                 % final time
    Ts = 0.01;              % sampling time
    t  = 0:Ts:tf;           % time vector
    N  = length(t);         % number of points of the simulation 
    n = 7;                  % joint number
    q  = zeros(n,N);        % q(:,i) collects the joint position for t(i)
    dq  = zeros(n,N);       % q(:,i) collects the joint position for t(i)
    % we can set initial configuration of the joints, but not the initial
    % position of the end-effector
    q(:,1) = [0 35  20 -45 30 30 -20]'/180*pi;   % initial configuration
    
    % << Initialization additional variables that I would like to use
    % inverse or transpose (of Jacobian)
    algorithm = 'transpose';
    % desired position
    x_pos = [0.65 0 0.4]';
    pos_error_t = zeros(3, N);
    % desired quaternion
    x_q = [0 0 0 1]';
    quat_error_t = zeros(3, N);
    
    if strcmp(algorithm, 'inverse')
        K = diag(10*ones(1, 6));
        % the larger K the larger robot movement
        % settling time will be 10*constant time (inverse of K diagonal values)
        dq_f = @(e, J_a) (pinv(J_a)*(K*e));
    else
        K = diag(20*ones(1, 6));
        dq_f = @(e, J_a) (J_a'*(K*e));
    end

    % >> End of my initialization block
    clc
    fprintf('----------------------');
    fprintf('\n simulation started ');
    fprintf('\n trying to connect...\n');
    [clientID, vrep ] = StartVrep(porta);
    
    handle_joint = my_get_handle_Joint(vrep,clientID);      % handle to the joints
    my_set_joint_target_position(vrep, clientID, handle_joint, q(:,1)); % first move to q0
    q_act(:,1) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep     

    % main simulation loop
    for i=2:N
        % << My code for calculating trajectory of the arm
        kuka_joint_temp = q(:, i-1);
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
        dq(:, i) = dq_f(error, J);
        % >> End of my code
        
        q(:,i) = q(:,i-1) + Ts*dq(:,i);
        my_set_joint_target_position(vrep, clientID, handle_joint, q(:,i));
        q_act(:,i) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep     
    end
    
    % << My code for checking of the errors
    % Errors converge go to zero in the end of the simulation
    figure;
    plot(t, pos_error_t);
    title('Position error');
    
    figure;
    plot(t, quat_error_t);
    title('Quaternion error');
    
    % << End of my code

    DeleteVrep(clientID, vrep); 
            
end

% constructor
function [clientID, vrep ] = StartVrep(porta)

    vrep = remApi('remoteApi');   % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1);        % just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1',porta,true,true,5000,5);% start the simulation
    
    if (clientID>-1)
        disp('remote API server connected successfully');
    else
        disp('failed connecting to remote API server');   
        DeleteVrep(clientID, vrep); %call the destructor!
    end
    % to change the simulation step time use this command below, a custom dt in v-rep must be selected, 
    % and run matlab before v-rep otherwise it will not be changed 
    % vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, 0.002, vrep.simx_opmode_oneshot_wait);
    
end  

% destructor
function DeleteVrep(clientID, vrep)

    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
%   vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
    vrep.simxFinish(clientID);  % close the line if still open
    vrep.delete();              % call the destructor!
    disp('simulation ended');
    
end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)
           
    [m,n] = size(q);
    for i=1:n
        for j=1:m
            err = vrep.simxSetJointTargetPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
            if (err ~= vrep.simx_error_noerror)
                fprintf('failed to send joint angle q %d \n',j);
            end
        end
    end
    
end

function handle_joint = my_get_handle_Joint(vrep,clientID)

    [~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint1',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint2',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint3',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint4',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint5',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint6',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)
           
    [~,n] = size(q);
    
    for i=1:n
        joints_positions = vrep.simxPackFloats(q(:,i)');
        [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);

        if (err~=vrep.simx_return_ok)   
           fprintf('failed to send the string signal of iteration %d \n',i); 
        end
    end
    pause(8);% wait till the script receives all data, increase it if dt is too small or tf is too high
    
end


function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)
    
    for j=1:n
         vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
    end

    pause(0.05);

    for j=1:n          
         [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
    end

    if (err(j)~=vrep.simx_return_ok)   
           fprintf(' failed to get position of joint %d \n',j); 
    end

end

