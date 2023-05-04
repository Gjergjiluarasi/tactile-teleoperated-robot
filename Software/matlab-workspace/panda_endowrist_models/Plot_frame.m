% Display of a 3D frame at a given location
% Written by Valentin Le Mesle
% 
% Input : R - rotation matrix describing the orientation of the frame
%         P - Vector describing the cartesian position of the frame
%         scale - Parameter allowing to change the length of the displayed
%         axes
%
% Note: Uncomment the lines 16 and 17 to create a figure 

function Plot_frame(R,P,scale)

%% Definition of the 3 axes of the frame
x_axis = [P,P+R(1:3,1)*scale];
y_axis = [P,P+R(1:3,2)*scale];
z_axis = [P,P+R(1:3,3)*scale];

%% Display of the 3 axis frame located in a desired position P
% figure
% hold on
plot3(P(1),P(2),P(3),'*')
plot3(x_axis(1,:),x_axis(2,:),x_axis(3,:),'r',"linewidth",2)
plot3(y_axis(1,:),y_axis(2,:),y_axis(3,:),'g',"linewidth",2)
plot3(z_axis(1,:),z_axis(2,:),z_axis(3,:),'b',"linewidth",2)

 end