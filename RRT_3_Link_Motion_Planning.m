clc
clear 

theta1_s = [0; 0; 30]; 
theta2_s = [0; -30; 30]; 
theta3_s = [0; -90;  30]; 

theta1_f = [60; 30; 90]; 
theta2_f = [30; 90; 0]; 
theta3_f = [30; 60; 0]; 

ob1_pos_x = [0.8; 0.8; 0.7];
ob1_pos_y = [0.4; 0.1; 0];
ob1_r = [0.1; 0.1; 0.1]; 

ob2_pos_x = [0; 0.5; 0.3];
ob2_pos_y = [0; 0.6; 0.6];
ob2_r = [0; 0.1; 0.1]; 


[arr1 p1] = myRRT(theta1_s(1),theta2_s(1), theta3_s(1), theta1_f(1), theta2_f(1), theta3_f(1),ob1_pos_x(1), ob1_pos_y(1), ob1_r(1),ob2_pos_x(1), ob2_pos_y(1), ob2_r(1));
[arr2 p2] = myRRT(theta1_s(2),theta2_s(2), theta3_s(2), theta1_f(2), theta2_f(2), theta3_f(2),ob1_pos_x(2), ob1_pos_y(2), ob1_r(2),ob2_pos_x(2), ob2_pos_y(2), ob2_r(2));
[arr3 p3] = myRRT(theta1_s(3),theta2_s(3), theta3_s(3), theta1_f(3), theta2_f(3), theta3_f(3),ob1_pos_x(3), ob1_pos_y(3), ob1_r(3),ob2_pos_x(3), ob2_pos_y(3), ob2_r(3));


function [G parent] = myRRT(theta1_s,theta2_s, theta3_s, theta1_f, theta2_f, theta3_f,ob1_pos_x, ob1_pos_y, ob1_r,ob2_pos_x, ob2_pos_y, ob2_r)
    
    L1 = 0.5; 
    L2 = 0.3; 
    L3 = 0.3; 
    L4 = 0.05; 
    N = 5000;

    %Generate points for arm
    Qstart = [theta1_s(1) theta2_s(1) theta3_s(1)];
    O0 = [0; 0]; 
    O1 = [O0(1)+L1*cosd(theta1_s(1)) O0(2)+L1*sind(theta1_s(1))];
    O2 = [O1(1)+L2*cosd(theta2_s(1)+theta1_s(1)) O1(2)+L2*sind(theta2_s(1)+theta1_s(1))];
    O3 = [O2(1)+L3*cosd(theta3_s(1)+theta2_s(1)+theta1_s(1)) O2(2)+L3*sind(theta3_s(1)+theta2_s(1)+theta1_s(1))];
    O0 = transpose(O0); 
    C1 = [ob1_pos_x(1) ob1_pos_y(1)];
    C2 = [ob2_pos_x(1) ob2_pos_y(1)];
    G=[   ];            % Array to store valid node
    G=[G;Qstart];
    parent=[1];
    found = 0; 
    
    %Calculate if there is intersection with arm
    delta1 = circle_line_intersection(O0,O1,C1,ob1_r);
    delta2 = circle_line_intersection(O1,O2,C1,ob1_r);
    delta3 = circle_line_intersection(O2,O3,C1,ob1_r);

    delta4 = circle_line_intersection(O0,O1,C2,ob2_r);
    delta5 = circle_line_intersection(O1,O2,C2,ob2_r);
    delta6 = circle_line_intersection(O2,O3,C2,ob2_r);

    figure
    pause off
    hold on
    circle(ob1_pos_x(1),ob1_pos_y(1),ob1_r(1));
    circle(ob2_pos_x(1),ob2_pos_y(1),ob2_r(1));
    axis equal
    
    for i=1:N
        %Generate Qrandom
        Qg = [theta1_f theta2_f theta3_f];
        coin=rand(1)*100;  
        if(coin<=99)
           theta1_rand=(rand(1)*360)-180;theta2_rand=(rand(1)*360)-180;theta3_rand=(rand(1)*360)-180;
           Qrand=[theta1_rand theta2_rand theta3_rand];  % Random Configuration, Qrand
        else
           Qrand=Qg;           % assign Qg to be Qrand with 1% probability 
           theta1_rand=Qg(1);theta2_rand = Qg(2);theta3_rand = Qg(3);                    % and try to connect to the tree
        end
        
        %Find Qnear
        O0_rand = [0; 0]; 
        O1_rand = [O0_rand(1)+L1*cosd(theta1_rand) O0_rand(2)+L1*sind(theta1_rand)];
        O2_rand = [O1_rand(1)+L2*cosd(theta2_rand+theta1_rand) O1_rand(2)+L2*sind(theta2_rand+theta1_rand)];
        O3_rand = [O2_rand(1)+L3*cosd(theta3_rand+theta2_rand+theta1_rand) O2_rand(2)+L3*sind(theta3_rand+theta2_rand+theta1_rand)];
        O0_rand = transpose(O0_rand);
    
        d=1000;
        [n,m]=size(G);
        Qnear=G(1,1:m);
        for j=1:n
    
            theta1_G = G(j,1);
            theta2_G = G(j,2);
            theta3_G = G(j,3);
    
            O0_G = [0 0]; 
            O1_G = [O0_G(1)+L1*cosd(theta1_G) O0_G(2)+L1*sind(theta1_G)];
            O2_G = [O1_G(1)+L2*cosd(theta2_G+theta1_G) O1_G(2)+L2*sind(theta2_G+theta1_G)];
            O3_G = [O2_G(1)+L3*cosd(theta3_G+theta2_G+theta1_G) O2_G(2)+L3*sind(theta3_G+theta2_G+theta1_G)];
            
            rho = sqrt((theta1_rand-theta1_G)^2+(theta2_rand-theta2_G)^2+(theta3_rand-theta3_G)^2);
            if(rho<d)
                Qnear=G(j,1:m);
                d=rho;
                near_node=j;
             end
        end
             
        % Find Qnew by interpolating
        t=0;
        tmax=0;
        steps=30;     
        t_step=1/steps;
        %linear interpolation between Qnear and Qrand
        %stepping from Qnear towards Qrand 
        while(t<1)
            theta1_temp=(1-t)*Qnear(1,1)+(t)*Qrand(1,1);
            theta2_temp=(1-t)*Qnear(1,2)+(t)*Qrand(1,2);
            theta3_temp=(1-t)*Qnear(1,3)+(t)*Qrand(1,3);
            Qtemp=[theta1_temp theta2_temp theta3_temp];
            %plot_arm(theta1_temp, theta2_temp, theta3_temp);
        
            c = world_col(theta1_temp,theta2_temp,theta3_temp,ob1_pos_x,ob1_pos_y,ob1_r,ob2_pos_x,ob2_pos_y,ob2_r);
            if(world_col(theta1_temp,theta2_temp,theta3_temp,ob1_pos_x,ob1_pos_y,ob1_r,ob2_pos_x,ob2_pos_y,ob2_r)==0)             % if no collision
                tmax=t;
                Qnew=Qtemp;
            else                                % if collision
                break
            end
                t=t+t_step;
        end
        
        G=[G;Qnew];% add node Qrand
        plot_arm(Qnew(1), Qnew(2), Qnew(3))
        parent=[parent;near_node]; 
        f = find(Qnew,Qg);

        if f == 1      % check if the added node is the goal node
            found=1;
            Qnew;
            Qg;
            break;        % terminate if goal node is added
        end
        
        %hold off
    end
    hold off
    
    
    
    if (found==1)
        figure
        pause on
        hold on
        circle(ob1_pos_x(1),ob1_pos_y(1),ob1_r(1));
        circle(ob2_pos_x(1),ob2_pos_y(1),ob2_r(1));
        axis equal
        plot_arm(Qg(1),Qg(2),Qg(3)); % plot goal node
        t=near_node;            % t holds the index of Qnear
        k=parent(t);            % k holds the index of the parent of Qnear
        ttmax = 1; 
        while(ttmax>=0)
            Qtt=[(1-ttmax)*G(t,1)+ttmax*Qg(1) (1-ttmax)*G(t,2)+ttmax*Qg(2) (1-ttmax)*G(t,3)+ttmax*Qg(3)];
            plot_arm(Qtt(1),Qtt(2),Qtt(3));
            pause(0.1)
            ttmax=ttmax-t_step;
        end
    
        pos = Qg;
        Qstart;
        back = 0;
        while back == 0
            plot_arm(G(t,1),G(t,2),G(t,3));
            ttmax = 1;
            while(ttmax>=0)
                Qtt=[(1-ttmax)*G(k,1)+ttmax*G(t,1) (1-ttmax)*G(k,2)+ttmax*G(t,2) (1-ttmax)*G(k,3)+ttmax*G(t,3)];
                plot_arm(Qtt(1),Qtt(2),Qtt(3));
                pause(0.1)
                ttmax=ttmax-t_step;
            end

            
            if pos(1) == Qstart(1) && pos(2) == Qstart(2) && pos(3) == Qstart(3)
                back = 1;
                plot_arm(G(k,1),G(k,2),G(k,3));
            end

            t=k;                % updating Qnear_new=parent of Qnear_old
            k=parent(t);        % updating parent of Qnear_new
            pos = [G(t,1) G(t,2) G(t,3)];
        end

    end
    pause off
    hold off
end

function h = circle(x,y,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
end

function delta = circle_line_intersection(A, B, C, R)
    % % For Mathematical Explanation: http://doswa.com/2009/07/13/circle-segment-intersectioncollision.html
    % INPUTS :
    % A and B are two end points of line segment
    % A: [x,y] coordinate of one end of the line segment
    % B: [x,y] coordinate of the other end of the line segment
    % C: [x,y] coordinate of center of the circle
    % R: radius of the circle
    % OUTPUT :
    % delta > 0 : no intersection
    % delta = 0 : tangent
    % delta < 0 : intersection
    
    BA = B - A; % vector from A towards B
    CA = C - A; % vector from A towards C
    proj = dot(CA, BA/norm(BA)); % length of projection of CA on BA
    % find point on the line closest to the circle
    if proj <= 0
        closest_point = A;
    elseif abs(proj - norm(BA)) < 0.000001 || proj - norm(BA) >0
        closest_point = B;
    else
        closest_point = A + proj * BA / norm(BA);
    end
    D = C - closest_point; % vector from closest point on line towards C
    delta = norm(D) - R;
end

function col = world_col(theta1, theta2, theta3, ob1_x, ob1_y, ob1_r,ob2_x, ob2_y, ob2_r)

    %
    L1 = 0.5; 
    L2 = 0.3; 
    L3 = 0.3; 
    L4 = 0.05; 

    col = 0; 
    O0 = [0; 0]; 
    O1 = [O0(1)+L1*cosd(theta1) O0(2)+L1*sind(theta1)];
    O2 = [O1(1)+L2*cosd(theta2+theta1) O1(2)+L2*sind(theta2+theta1)];
    O3 = [O2(1)+L3*cosd(theta3+theta2+theta1) O2(2)+L3*sind(theta3+theta2+theta1)];
    O4 = [O3(1)+L4*cosd(theta3+theta2+theta1+90) O3(2)+L4*sind(theta3+theta2+theta1+90)];
    O4_2 = [O3(1)+L4*cosd(theta3+theta2+theta1-90) O3(2)+L4*sind(theta3+theta2+theta1-90)];
    O0 = transpose(O0); 
    C1 = [ob1_x ob1_y];
    C2 = [ob2_x ob2_y];

    %Calculate if there is any intersection with arm and the obstacle
    delta1 = circle_line_intersection(O0,O1,C1,ob1_r);
    delta2 = circle_line_intersection(O1,O2,C1,ob1_r);
    delta3 = circle_line_intersection(O2,O3,C1,ob1_r);
    delta4 = circle_line_intersection(O3,O4,C1,ob1_r);
    delta5 = circle_line_intersection(O3,O4_2,C1,ob1_r);
    
    delta6 = circle_line_intersection(O0,O1,C2,ob2_r);
    delta7 = circle_line_intersection(O1,O2,C2,ob2_r);
    delta8 = circle_line_intersection(O2,O3,C2,ob2_r);
    delta9 = circle_line_intersection(O3,O4,C2,ob2_r);
    delta10 = circle_line_intersection(O3,O4_2,C2,ob2_r);
    
    if delta1 < 0 || delta2 < 0 || delta3 < 0 || delta4 < 0 || delta5 < 0 || delta6 < 0 || delta7 < 0 || delta8 < 0 || delta9 < 0 || delta10 < 0
        col = 1;
    end

end

function plot_arm(theta1, theta2, theta3)


    L1 = 0.5; 
    L2 = 0.3; 
    L3 = 0.3; 
    L4 = 0.05; 

    O0 = [0 0]; 
    O1 = [O0(1)+L1*cosd(theta1) O0(2)+L1*sind(theta1)];
    O2 = [O1(1)+L2*cosd(theta2+theta1) O1(2)+L2*sind(theta2+theta1)];
    O3 = [O2(1)+L3*cosd(theta3+theta2+theta1) O2(2)+L3*sind(theta3+theta2+theta1)];
    O4 = [O3(1)+L4*cosd(theta3+theta2+theta1+90) O3(2)+L4*sind(theta3+theta2+theta1+90)];
    O4_2 = [O3(1)+L4*cosd(theta3+theta2+theta1-90) O3(2)+L4*sind(theta3+theta2+theta1-90)];


    %plot([O0(1) O1(1) O2(1) O3(1)],[O0(2) O1(2) O2(2) O3(2)],"-b")
    plot([O0(1) O1(1)],[O0(2) O1(2)],"-b")
    plot([O1(1) O2(1)],[O1(2) O2(2)],"-r")
    plot([O2(1) O3(1)],[O2(2) O3(2)],"-g")
    plot([O3(1) O4(1)],[O3(2) O4(2)],"-k")
    plot([O3(1) O4_2(1)],[O3(2) O4_2(2)],"-k")
   

end

function line_int = line_col(p1_a,p2_a,p1_b,p2_b)


%Generate 1st line 
x1_a = p1_a(1);
y1_a = p1_a(2);
x2_a = p2_a(1);
y2_a = p2_a(2);

m_a = (y2_a-y1_a)/(x2_a-x1_a);
b_a = y1_a-m_a*x1_a;


%Generate 2nd line
x1_b = p1_b(1);
y1_b = p1_b(2);
x2_b = p2_b(1);
y2_b = p2_b(2);

m_b = (y2_b-y1_b)/(x2_b-x1_b);
%y=mx+b
b_b = y1_b-m_b*x1_b;

%Find x and y where the 2 lines intersect
x_int = (b_b-b_a)/(m_a-m_b);
y_int = m_a*x_int+b_a;

%Find if that x,y intersect value occurs in the line segment
max_x_a = max(x1_a,x2_a);
min_x_a = min(x1_a,x2_a);

max_x_b = max(x1_b,x2_b);
min_x_b = min(x1_b,x2_b);

if x_int < max_x_a && x_int > min_x_a && x_int < max_x_b && x_int > min_x_b
    line_int = 1;
else 
    line_int =0; 
end

end

function f = find(Q1,Qg)
    
    theta1 = Q1(1);
    theta2 = Q1(2);
    theta3 = Q1(3);

    theta1_goal = Qg(1);
    theta2_goal = Qg(2);
    theta3_goal = Qg(3);
    
    delta1 = abs(theta1_goal-theta1);
    delta2 = abs(theta2_goal-theta2);
    delta3 = abs(theta3_goal-theta3);
    
    req = 0.01;
    if delta1 < req && delta2 < req && delta3 < req
        f = 1;
    else 
        f = 0; 
    end
    
end
