function HW4_Problem1b()
% demo simulation of a bar hitting the ground
% system states: X = ['x';'theta';'dx';'dtheta'];
% control input: u = T;

%% Declare constants
clc; clear all; close all;
global params;

M = 1;
m = 0.2; 
l = 0.3; 
g = 9.81 ;  
params.M=M;
params.m=m;
params.l=l;
params.g =g;

%% Run Simulation
X0=[0;pi/6;0;0]; % X0 is the intial state of the system
% i. Simulate the system in 2 seconds
tspan=[0; 2]; % simulation time
% phase 1 (before impact)
options = optimoptions('quadprog','Display','off');
[t,X]=ode45(@robot_dynamics1,tspan,X0); % simulate the system with the robot dynamics for phase 1

anim(t,X,1/24); % animate the system and save the simulation video

% recreate control inputs
for i=1:length(t)
     u(:,i)=controller(t(i),X(i,:)');
end


%% Show plots of x(t), y(t), and theta(t) of the COM position and orientation over time.
% plot the simulation data
figure; 
plot(t,X(:,1)); legend('x'); title('plot of x(t) over time'); % Plot system states from matlab
figure; 
plot(t,X(:,2)); legend('theta'); title('plot of theta(t) over time'); % Plot system states from matlab
figure; 
plot(t,u); legend('u'); title('control input');
end

%% Robot Dynamics 1
function dX1=robot_dynamics1(t1,X1)
global params;
u=controller(t1,X1);
M = params.M;
m = params.m;
l = params.l;
g = params.g;
x = X1(1);
theta = X1(2);
dx = X1(3); 
dtheta = X1(4); 

D =[          M + m, -l*m*cos(theta);
-l*m*cos(theta),           l^2*m];

N= [dtheta^2*l*m*sin(theta);
      -g*l*m*sin(theta)];

ddq_out = inv(D)*(-u-N); 
ddx = ddq_out(1); 
ddtheta = ddq_out(2); 

dX1 = [dx;dtheta;ddx;ddtheta];
end


%% Controller
function u=controller(t,X)
global params
M = params.M;
m = params.m;
l = params.l;
g = params.g;
x = X(1);
theta = X(2);
dx = X(3); 
dtheta = X(4);

% Linearization 
theta_d = 0; 
x_d = 0; 
dtheta_d = 0; 
dx_d = 0; 
dq_d = [dx_d;dtheta_d]; 
u_d = [0;0];

A =[0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                0, 1,                                                                           0;
0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                0, 0,                                                                              1;
0,                                                                                     (g*l*m*cos(theta_d)^2)/(- l*m*cos(theta_d)^2 + M*l + l*m) - (dtheta_d^2*l*m*cos(theta_d))/(- m*cos(theta_d)^2 + M + m) - (g*l*m*sin(theta_d)^2)/(- l*m*cos(theta_d)^2 + M*l + l*m) - (2*m*cos(theta_d)*sin(theta_d)*(- l*m*sin(theta_d)*dtheta_d^2 + x_d))/(- m*cos(theta_d)^2 + M + m)^2 - (2*g*l^2*m^2*cos(theta_d)^2*sin(theta_d)^2)/(- l*m*cos(theta_d)^2 + M*l + l*m)^2, 0,                    -(2*dtheta_d*l*m*sin(theta_d))/(- m*cos(theta_d)^2 + M + m);
0, (g*l*m*cos(theta_d)*(M + m))/(- l^2*m^2*cos(theta_d)^2 + l^2*m^2 + M*l^2*m) - (dtheta_d^2*l*m*cos(theta_d)^2)/(- l*m*cos(theta_d)^2 + M*l + l*m) - (sin(theta_d)*(- l*m*sin(theta_d)*dtheta_d^2 + x_d))/(- l*m*cos(theta_d)^2 + M*l + l*m) - (2*l*m*cos(theta_d)^2*sin(theta_d)*(- l*m*sin(theta_d)*dtheta_d^2 + x_d))/(- l*m*cos(theta_d)^2 + M*l + l*m)^2 - (2*g*l^3*m^3*cos(theta_d)*sin(theta_d)^2*(M + m))/(- l^2*m^2*cos(theta_d)^2 + l^2*m^2 + M*l^2*m)^2, 0, -(2*dtheta_d*l*m*cos(theta_d)*sin(theta_d))/(- l*m*cos(theta_d)^2 + M*l + l*m)];
 
B =[                                          0, 0;
                                              0, 0;
                 1/(- m*cos(theta_d)^2 + M + m), 0;
cos(theta_d)/(- l*m*cos(theta_d)^2 + M*l + l*m), 0];

Q = diag([10,10,10,10]);
R = 1;
K = lqr(A,B,Q,R);

X = [x;theta;dx;dtheta]; 
u_LQR = K*X;

H = 2*eye(2);
f = -2*u_LQR;
A1 = [1 0;-1 0];
b = [10;10];
options = optimoptions('quadprog','Display','off');
[u_QP,fval,exit,out] = quadprog(H,f,A1,b,[],[],[],[],[],options);
u = u_QP; % you can put your controller here
end

%% Animation
function anim(t,x,ts)
[te,xe]=even_sample(t,x,1/ts);

figure(1);

axes1 = axes;

%save as a video
% Create animation for the system and attach the video link into your HW.
spwriter = VideoWriter('video_HW4_Prob1b.avi');
set(spwriter, 'FrameRate', 1/ts,'Quality',100);
open(spwriter);

fig1 = figure(1);
axis equal

figure_x_limits = [-1 1];
figure_y_limits = [-0.5 0.5];


axes1 = axes;
set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
set(axes1,'Position',[0 0 1 1]);
set(axes1,'Color','w');

for k = 1:length(te)
    drawone(axes1, xe(k,:)');
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
    drawnow
    pause(ts);
    frame = getframe(gcf);
    writeVideo(spwriter, frame);
end

end

function [Et, Ex] = even_sample(t, X, Fs)
%       CONVERTS A RANDOMLY SAMPLED SIGNAL SET INTO AN EVENLY SAMPLED
%       SIGNAL SET (by interpolation)
% Obtain the process related parameters
N = size(X, 2);    % number of signals to be interpolated
M = size(t, 1);    % Number of samples provided
t0 = t(1,1);       % Initial time
tf = t(M,1);       % Final time
EM = (tf-t0)*Fs;   % Number of samples in the evenly sampled case with
                   % the specified sampling frequency
Et = linspace(t0, tf, EM)';

% Using linear interpolation (used to be cubic spline interpolation)
% and re-sample each signal to obtain the evenly sampled forms
for s = 1:N
  Ex(:,s) = interp1(t(:,1), X(:,s), Et(:,1)); 
end
end

%% Draw Robot
function drawone(parent, X)
% draw the robot at the current frame
global params
tem = get(parent,'Children');
delete(tem);
M = params.M;
m = params.m;
l = params.l;
g = params.g;
box_y = 0.1; 
box_x = 0.1; 

% Get coordinates for each end of the rod
O1 = [X(1), box_y];
O2 = [X(1) - l*sin(X(2)), box_y + l*cos(X(2))]; 

% Draw a box around the COM. The box orientation will change wrt the
% orientation of the body. 
box_p1 = O1 + [box_x, 0];
box_p2 = O1 + [box_x, -box_y];
box_p3 = O1 + [-box_x, 0];
box_p4 = O1 + [-box_x, -box_y];


% Plot the rod
yline(0); 
hold on; 
line([O1(1) O2(1)],[O1(2) O2(2)],'Color','b');


% Draw box
line([box_p1(1) box_p2(1)],[box_p1(2) box_p2(2)],'Color','r');
line([box_p2(1) box_p4(1)],[box_p2(2) box_p4(2)],'Color','r');
line([box_p3(1) box_p4(1)],[box_p3(2) box_p4(2)],'Color','r');
line([box_p3(1) box_p1(1)],[box_p3(2) box_p1(2)],'Color','r');
      
end