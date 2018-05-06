% Initialization file for the direct kinematics exercise 
% Kuka LWR DH
% a(m), alpha, d(m), theta
kuka_lwr = [0 pi/2 0.3105; %1
    0 -pi/2 0; %2
    0 pi/2 0.4; %3
    0 -pi/2 0; %4
    0 pi/2 0.39; %5
    0 -pi/2 0; %6
    0 0 0.078]; %7

% initial e. e. position
pa = [0.65 0 0.4];
% initial orientation to be found
% approach vector is aligned with
x = [1 -1 0];
% the e.e. is required to advance 5 cm in the direction of ˆx
% while rotating 4pi rad
%theta_kuka = [0 0 0 0 0 0 0]';
theta_kuka = [0 -45  0 -90 0 45 0]'/180*pi;

q_initial = [-0.1201
    -0.7300
    0.3238
   -1.4300
    1.0247
    1.1270
    0.6155];
% 
%    -0.1201
%    -0.7300
%     0.3238
%    -1.4300
%     1.0247
%     1.1270
%     0.6155

DH_kuka = [kuka_lwr theta_kuka];

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
