function DrawRobot(DH)
%
% Draw a robot in 3D with intermediate frames
%
% function DrawRobot(DH)
%
% input:
%       DH     dim nx4     Denavit-Hartenberg table

n = size(DH,1);
d = 0.03;

T0 = zeros(4,4,n);
T0(:,:,1) = Homogeneous(DH(1,:));
for i=2:n
    T0(:,:,i) = T0(:,:,i-1)*Homogeneous(DH(i,:));
end

hold on
view([2 -2 2])
T00 = [eye(3) [0 0 0]'; 0 0 0 1];
DrawFrame(T00,1);
for i=1:n
    DrawFrame(T0(:,:,i),0);
end

DrawLink(T00(1:3,4),T0(1:3,4,1),d)
for i=2:n
    DrawLink(T0(1:3,4,i-1),T0(1:3,4,i),d)
end
axis equal
grid
xlabel('x')
ylabel('y')
zlabel('z')