% Initialization file for the direct kinematics exercise 

% Link lengths
a1 = 1;
a2 = .4;
a3 = .2;

% DH parameters table for the two-link planar arm
DH_planar2link = [a1 0 0;
      a2 0 0];
     
% DH parameters table for the three-link planar arm
DH_planar3link = [a1 0 0;
      a2 0 0;
      a3 0 0];
  
% DH parameters table for the anthropomorphic arm
DH_anthrop = [a1 pi/2 0;
          a2 0 0;
          a3 0 0];
      
% Joint variable vector (delete the last element 
% if you choose the two-link planar arm)
theta = [0 pi/4 pi/4]';
% For planar 
% theta = [0 pi/4]';


% Final DH parameters table (replace the first element with the DH parametes table relative to the chosen arm)
DH = [DH_planar3link theta];
