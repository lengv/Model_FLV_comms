function [stability, stable, lambda] = check_stability(vertecies, points, err)

% Description:
%   Checks if points are within the stability region of triangle defined by
%   vertices

% Inputs:
%   vertecies   - vertecies of stability region
%   points      - points to be considered (ZMP position)
%   err         - error or factor of safety

% Outputs:
%   stability   - degree of stability <=0.33^2, 0 is on the edge, <0 is
%                 unstable
%   stable      - boolean, 1 for stable. Includes err in calculation
%   lambda      - coordinates

% Details:
%   Uses a formulation of Barycentric coordinates specifically for
%   triangles. This can be generalised to any polygon if necessary. The
%   coordinates of the Barycentric system (a,b,c) is used to calculate
%   stability.

if nargin == 2
    err = 0;
end

[r, c] = size(vertecies);
if c ~= 3 
    error('Number of vertecies not equal to 3');
end

if r ~= 2 && r~= 3;
    error('Each vertex requires at least 2 points (x,y) and a maximum of 3 (x,y,z)');
end

[r, c] = size(points);
if r ~= 2 && c ~= 0 && r~=3
    error('Points requires at least 2 rows and max 3');
end

% Find Barycentric coordinates
det = (vertecies(2,2)-vertecies(2,3)).*(vertecies(1,1)-vertecies(1,3))+(vertecies(1,3)-vertecies(1,2)).*(vertecies(2,1)-vertecies(2,3));

a = ( ( vertecies(2,2)-vertecies(2,3)).*(points(1,:)-vertecies(1,3) ) + ( vertecies(1,3)-vertecies(1,2)).*(points(2,:)-vertecies(2,3) ) )./det;
b = ( ( vertecies(2,3)-vertecies(2,1)).*(points(1,:)-vertecies(1,3) ) + ( vertecies(1,1)-vertecies(1,3)).*(points(2,:)-vertecies(2,3) ) )./det;
c = 1-a-b;

lambda = [a;b;c];

% if a*b*c < 0 then outside
%          = 0 then on line
%          > 0 inside triangle up to 1

if any(lambda <= 0) || any(lambda >= 1)
    stability = 0;
else 
    stability = (1-any(lambda <= 0))*(a.*b.*c./((1/3)^3));
end

stable = stability > 0+err;

end