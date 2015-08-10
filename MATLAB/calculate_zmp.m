function [x, y, z] = calculate_zmp(A,I,R,w,w_dot,m,r)

% Inputs:
%   a       - acceleration
%   I       - moment of inertia tensor
%   R       - rotation matrix
%   w       - angular velocity
%   w_dot   - angular acceleration
%   m       - mass
%   r       - position vector of CoG

% rotated accelerations
% A = R*A;
% A(1:2) = -A(1:2);

% for ease
Iw_dot = I*w_dot;
Iw = I*w;

% wxIw = cross_(w,I*w);
wxIw = [w(2,:).*Iw(3,:)-w(3,:).*Iw(2,:);...
     w(3,:).*Iw(1,:)-w(1,:).*Iw(3,:);...
     w(1,:).*Iw(2,:)-w(2,:).*Iw(1,:)];

% Note that A is negative for A(1) and A(3) because they are reactionary 
% TODO this should be incorporated into the calculation of A and not done
% in here to maintain A as accels for d'lambert forces
x = (m*(r(1)*A(3)-r(3)*-A(1)) - sum(Iw_dot(2,:))+wxIw(2)) / (m*A(3));
y = (m*(r(2)*A(3)-r(3)*-A(2)) + sum(Iw_dot(1,:))+wxIw(1)) / (m*A(3));
z = 0;

end
