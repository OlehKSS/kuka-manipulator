function T = DirectKinematics(DH)
import kinematics.Homogeneous;
%DH matrix of Denavit-Hartenberg parameters
%returns end-effector position
    
    [chain_length, ~] = size(DH);
    T = cell(1, chain_length);
    T{1} = Homogeneous(DH(1, :));
    
    for i=2:chain_length
        T{i} = T{i - 1}*Homogeneous(DH(i, :));
    end    
end