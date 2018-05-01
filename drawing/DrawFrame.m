function DrawFrame(T,mytext)
%
% DrawFrame(T,mytext)
%
% input:
%   T   dim 4x4    homogeneous transf. matrix
%   mytext          ==1 writes "x", "y", "z" close to the versor
%
% G. Antonelli, Sistemi Robotici, fall 2012

scala = .1;     % scaling factor with respect to the versor

d_ax = .05*scala; % axis diameter
d_ar = .1*scala;  % axis diameter of the arrow head
s_ar = .7;        % normalized position of the beginning of the head arrow
d_txt = 1.1;      % text scaling factor

T(1:3,1:3) = scala*T(1:3,1:3);

% axis x - red
arrow3d([T(1,4) T(1,4)+T(1,1)],[T(2,4) T(2,4)+T(2,1)],[T(3,4) T(3,4)+T(3,1)],s_ar,d_ax,d_ar,[1 0 0]);
if mytext==1
    text(d_txt*(T(1,4)+T(1,1)),d_txt*T(2,4)+T(2,1),d_txt*T(3,4)+T(3,1),'x');
end
% axis y - green
arrow3d([T(1,4) T(1,4)+T(1,2)],[T(2,4) T(2,4)+T(2,2)],[T(3,4) T(3,4)+T(3,2)],s_ar,d_ax,d_ar,[0 1 0]);
if mytext==1
    text(d_txt*(T(1,4)+T(1,2)),d_txt*T(2,4)+T(2,2),d_txt*T(3,4)+T(3,2),'y');
end
% axis z - blue
arrow3d([T(1,4) T(1,4)+T(1,3)],[T(2,4) T(2,4)+T(2,3)],[T(3,4) T(3,4)+T(3,3)],s_ar,d_ax,d_ar,[0 0 1]);
if mytext==1
    text(d_txt*(T(1,4)+T(1,3)),d_txt*T(2,4)+T(2,3),d_txt*T(3,4)+T(3,3),'z');
end
