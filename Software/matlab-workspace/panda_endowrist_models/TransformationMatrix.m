% Computation an homogeneous transformatrion matrix using MDH Parameters
% Written by Valentin Le Mesle
% 
% Input : theta - rotation around the z axis of the current link
%         alpha - Rotation around x of the PREVIOUS link
%         a     - Translation along x of the PREVIOUS link
%         d     - Translation along z of the current link
%
% Note: Displacement sequence for MDH transformation - Rot(x), Tra(x),
%       Rot(z), Tra(z)

function T = TransformationMatrix(theta,alpha,a,d)

T = [cos(theta), -sin(theta), 0, a;...
     sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);...
     sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);...
     0,0,0,1];
end
