clear 

% List robot parameters
d1 = 0.1;  
d2 = 0; 
d3 = 0; 

a1 = 0;
a2 = 0.2;
a3 = 0.2;

theta1_offset = 0; 
theta2_offset = 0; 
theta3_offset = 0; 

theta1_input = [pi/6+pi 0     pi/6    pi]; 
theta2_input = [1.3102-pi  pi/4    pi/2]; 
theta3_input = [-pi/4 pi/4  -pi/2   pi/2];
%theta 2 = alpha - beta

% theta1_input = [pi/6+0 0     pi/6    pi]; 
% theta2_input = [-1.3102  pi/4    pi/2]; 
% theta3_input = [pi/4 pi/4  -pi/2   pi/2];
%theta 2 = alpha - beta

%-pi/6
s = 0.393-d1;
r = sqrt(0.195^2+0.112^2);
t3_a = acos((s^2+r^2-a2^2-a3^2)/(2*a2*a3))
-theta3_input(1)
beta = atan(a3*sin(t3_a)/(a2+a3*cos(t3_a)));
alpha = atan(s/r);
t2 = -(alpha + beta)+pi
-theta2_input(1)

% Generate arm for each configuration
for i = 1:1
   
    theta1 = theta1_offset+theta1_input(i);
    theta2 = theta2_offset+theta2_input(i);
    theta3 = theta3_offset+theta3_input(i);
    
    alpha1 = -pi/2; 
    alpha2 = 0;
    alpha3 = 0; 
    
    % Calculate each homogeneous transformation
    A1 = [cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1); 
        sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1); 
        0 sin(alpha1) cos(alpha1) d1;
        0 0 0 1];
    
    A2 = [cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2) a2*cos(theta2); 
        sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2) a2*sin(theta2); 
        0 sin(alpha2) cos(alpha2) d2;
        0 0 0 1]; 
    
    A3 = [cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3); 
        sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3) a3*sin(theta3); 
        0 sin(alpha3) cos(alpha3) d3;
        0 0 0 1];  
    
    % Calculate the end of the arm in the 0 frame
    A0to3 = A1*A2*A3; 
    
    A0to1 = A1;
    A0to2 = A1*A2;
    A0to3 = A1*A2*A3;
    
    % Get each origin point of each link
    p1 = round(A0to1(1:3,4),3);
    p2 = round(A0to2(1:3,4),3);
    p3 = round(A0to3(1:3,4),3);
    
    %Plot each link
    figure(i); 
    plot3([0 p1(1)],[0 p1(2)],[0 p1(3)])
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    hold on 
    plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)])
    plot3([p2(1) p3(1)],[p2(2) p3(2)],[p2(3) p3(3)])


    % Draw Axes for each origins
    %O0

    Xax_0 = 0 + 1*[0.02; 0; 0];
    Yax_0 = 0 + 1*[0; 0.02; 0];
    Zax_0 = 0 + 1*[0; 0; 0.02];

    h1_0 = plot3([0 Xax_0(1)],[0 Xax_0(2)],[0 Xax_0(3)],'DisplayName','X Axis');
    h1_0.Color = 'b'; % x is blue arrows

    h2_0 = plot3([0 Yax_0(1)],[0 Yax_0(2)],[0 Yax_0(3)],'DisplayName','Y Axis');
    h2_0.Color = 'r'; % y is red arrows

    h3_0 = plot3([0 Zax_0(1)],[0 Zax_0(2)],[0 Zax_0(3)],'DisplayName','Z Axis');
    h3_0.Color = 'g'; % z is green arrows

    %O1
    Xax_1 = p1 + A0to1(1:3,1:3)*[0.02; 0; 0];
    Yax_1 = p1 + A0to1(1:3,1:3)*[0; 0.02; 0];
    Zax_1 = p1 + A0to1(1:3,1:3)*[0; 0; 0.02];

    h1_1 = plot3([p1(1) Xax_1(1)],[p1(2) Xax_1(2)],[p1(3) Xax_1(3)],'DisplayName','X Axis');
    h1_1.Color = 'b'; % x is blue arrows

    h2_1 = plot3([p1(1) Yax_1(1)],[p1(2) Yax_1(2)],[p1(3) Yax_1(3)],'DisplayName','Y Axis');
    h2_1.Color = 'r'; % y is red arrows

    h3_1 = plot3([p1(1) Zax_1(1)],[p1(2) Zax_1(2)],[p1(3) Zax_1(3)],'DisplayName','Z Axis');
    h3_1.Color = 'g'; % z is green arrows
    
    %O2
    Xax_2 = p2 + A0to2(1:3,1:3)*[0.02; 0; 0];
    Yax_2 = p2 + A0to2(1:3,1:3)*[0; 0.02; 0];
    Zax_2 = p2 + A0to2(1:3,1:3)*[0; 0; 0.02];

    h1_2 = plot3([p2(1) Xax_2(1)],[p2(2) Xax_2(2)],[p2(3) Xax_2(3)],'DisplayName','X Axis');
    h1_2.Color = 'b'; % x is blue arrows

    h2_2 = plot3([p2(1) Yax_2(1)],[p2(2) Yax_2(2)],[p2(3) Yax_2(3)],'DisplayName','Y Axis');
    h2_2.Color = 'r'; % y is red arrow

    h3_2 = plot3([p2(1) Zax_2(1)],[p2(2) Zax_2(2)],[p2(3) Zax_2(3)],'DisplayName','Z Axis');
    h3_2.Color = 'g'; % z is green arrows


    %O3
    Xax = p3 + A0to3(1:3,1:3)*[0.02; 0; 0];
    Yax = p3 + A0to3(1:3,1:3)*[0; 0.02; 0];
    Zax = p3 + A0to3(1:3,1:3)*[0; 0; 0.02];

    h1 = plot3([p3(1) Xax(1)],[p3(2) Xax(2)],[p3(3) Xax(3)],'DisplayName','X Axis');
    h1.Color = 'b'; % x is blue arrows

    h2 = plot3([p3(1) Yax(1)],[p3(2) Yax(2)],[p3(3) Yax(3)],'DisplayName','Y Axis');
    h2.Color = 'r'; % y is red arrows

    h3 = plot3([p3(1) Zax(1)],[p3(2) Zax(2)],[p3(3) Zax(3)],'DisplayName','Z Axis');
    h3.Color = 'g'; % z is green arrows
    legend([h1,h2,h3]);

    axis equal
    hold off
    
    view(3)
end

