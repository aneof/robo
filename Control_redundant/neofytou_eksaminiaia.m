%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% KINEMATIC CONTROL of REDUNDANT ROBOT MANIPULATOR %%%%%
%%%%%  (PLANAR 3 DOF MANIPULATOR WITH 1 REDUNDANT DOF) %%%%%

close all;
clear all;

%%Gain for control of redundant degrees of freedom 
kc = 10;  
%kc= 0;   
%kc= 20;

%% Length of the links




%% *** Sampling period ***
%% *** for the robot motion, kinematic simulation:
dt = 0.001; %dt = 0.001; i.e. 1 msec) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% *** Create (or load from file) reference signals ***
%% *** DESIRED MOTION PROFILE - TASK SPACE ***
Tf=1.0; 	%  duration of motion (in secs)
t=0:dt:Tf;


% Example of desired trajectory : linear segment (x0,y0)-->(xf,yf); Time duration: Tf;
disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...'); %%
disp(' '); 

%% *** Initial configuration *** 
qd1_0 = 0; qd2_0 = 0; qd3_0 = 0;
qd4_0 = 0; qd5_0 = 0; qd6_0 = 0;
qd7_0 = 0; qd8_0 = 0; qd9_0 = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% *** 1st subtask: desired trajectory *** 
% *** (xd2_0,yd2_0),(xd3_0,yd3_0),(xde_0,yde_0): initial positions 
%     for positions of joints 2, 3, and end-effector, respecitvely
% *** (xde_f,yde_f): final end-effector position
s1 = sin(qd1_0); 
s12 = sin(qd1_0+qd2_0); 
s123 = sin(qd1_0+qd2_0+qd3_0);
s1234 = sin(qd1_0+qd2_0+qd3_0+qd4_0);
s12345 = sin(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0);
s123456 = sin(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0);
s1234567 = sin(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0+qd7_0);
s12345678 = sin(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0+qd7_0+qd8_0);
s123456789 = sin(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0+qd7_0+qd8_0+qd9_0);
c1 = cos(qd1_0); 
c12 = cos(qd1_0+qd2_0); 
c123 = cos(qd1_0+qd2_0+qd3_0);
c1234 = cos(qd1_0+qd2_0+qd3_0+qd4_0);
c12345 = cos(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0);
c123456 = cos(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0);
c1234567 = cos(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0+qd7_0);
c12345678 = cos(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0+qd7_0+qd8_0);
c123456789 = cos(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0+qd7_0+qd8_0+qd9_0);

%%% xd kai yd einai ta px kai py 
xd2_0=-s1;	%%% initial positions
yd2_0=c1;
xd3_0=-s1+c12;
yd3_0=c1+s12;
xd4_0=-s1+c12-s123;
yd4_0=c1+s12+c123;
xd5_0=-s1+c12-s123-s1234;
yd5_0=c1+s12+c123+c1234;
xd6_0=-s1+c12-s123-s1234+c12345;
yd6_0=c1+s12+c123+c1234+s12345;
xd7_0=-s1+c12-s123-s1234+c12345+s123456;
yd7_0=c1+s12+c123+c1234+s12345-c123456;
xd8_0=-s1+c12-s123-s1234+c12345+s123456+s1234567;
yd8_0=c1+s12+c123+c1234+s12345-c123456-c1234567;
xd9_0=-s1+c12-s123-s1234+c12345+s123456+s1234567+c12345678;
yd9_0=c1+s12+c123+c1234+s12345-c123456-c1234567+s12345678;
xdE_0=-s1+c12-s123-s1234+c12345+s123456+s1234567+c12345678+c123456789;
ydE_0=c1+s12+c123+c1234+s12345-c123456-c1234567+s12345678+s123456789;

xdE_f=xdE_0+3.8;	%%% final position
ydE_f=ydE_0;

%% desired trajectory (1st subtask): position (xd,yd); velocity (xd_,yd_) 
lambda_x = (xdE_f-xdE_0)/Tf;
lambda_y = (ydE_f-ydE_0)/Tf;
xd(1) = xdE_0;
yd(1) = ydE_0;
xd_(1)=lambda_x;
yd_(1)=lambda_y;

Nmax=Tf/dt + 1;
for k=2:Nmax;
   xd(k) = xd(k-1) + lambda_x*dt;
   yd(k) = yd(k-1) + lambda_y*dt;
   
   xd_(k)= lambda_x;
   yd_(k)= lambda_y;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%% FIGURE FOR ANIMATION %%%%%%%%%%%%
%%%%%%% (moving robot stick diagram) %%%%%%%
fig0 = figure;

%%% GUInterface: control buttons to move obstacle
d_safe = 20;
h1 = uicontrol('Style', 'pushbutton', 'String', 'Up',...
   'Position', [10 160 45 40],...
   'Callback', 'yobst=yobst+0.1;'); %%deletee
h2 = uicontrol('Style', 'pushbutton', 'String', 'Down',...
   'Position', [10 120 45 40],...
   'Callback', 'yobst=yobst-0.1;'); %%delete
h1 = uicontrol('Style', 'pushbutton', 'String', 'Right',...
   'Position', [10 80 45 40],...
   'Callback', 'xobst=xobst+0.1;'); %%deletee
h2 = uicontrol('Style', 'pushbutton', 'String', 'Left',...
   'Position', [10 40 45 40],...
   'Callback', 'xobst=xobst-0.1;'); %%delete
%%%%%%%%% START PLOTING FIGURE %%%%%%%%%%%%%%%
figure(fig0);
axis([-2 8 -5 5])
axis on
hold on

xlabel('x (m)');
ylabel('y (m)');
plot(xd, yd, 'm:');
dtk=50; %% interval between consecutive plots <---***

plot([0], [0], 'o');

%% OBSTACLE
xobst = 3.8;
yobst = 0;
rectangle('Position',[xobst,yobst+1.5,0.5,0.5],...
  'Curvature',[1,1], 'FaceColor','r')
rectangle('Position',[xobst,yobst,0.5,0.5],...
  'Curvature',[1,1], 'FaceColor','r')
obst1x=4.05;
obst1y=0.25;
obst2x=4.05;
obst2y=1.75;
obst = [obst1x obst1y ; obst2x obst2y];

plot([0, xd2_0], [0, yd2_0],'r');
plot([xd2_0], [yd2_0], '*');
plot([xd2_0, xd3_0], [yd2_0, yd3_0],'m');
plot([xd3_0], [yd3_0], '*');
plot([xd3_0, xd4_0], [yd3_0, yd4_0],'r');
plot([xd4_0], [yd4_0], '*');
plot([xd4_0, xd5_0], [yd4_0, yd5_0],'m');
plot([xd5_0], [yd5_0], '*');
plot([xd5_0, xd6_0], [yd5_0, yd6_0],'r');
plot([xd6_0], [yd6_0], '*');
plot([xd6_0, xd7_0], [yd6_0, yd7_0],'m');
plot([xd7_0], [yd7_0], '*');
plot([xd7_0, xd8_0], [yd7_0, yd8_0],'r');
plot([xd8_0], [yd8_0], '*');
plot([xd8_0, xd9_0], [yd8_0, yd9_0],'m');
plot([xd9_0], [yd9_0], '*');
plot([xd9_0, xdE_0], [yd9_0, ydE_0],'r');
plot([xdE_0], [ydE_0], 'y*');
plot([xdE_0], [ydE_0], 'g+');



box_x=0.8;  %%current box position 
box_y=-0.2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ****** KINEMATIC SIMULATION - Main loop ******
disp('Kinematic Simulation ...'); %%
disp(' '); %%

tt=0; tk=1;
qd(tk,1)=qd1_0; qd(tk,2)=qd2_0; qd(tk,3)=qd3_0;
qd(tk,4)=qd4_0; qd(tk,5)=qd5_0; qd(tk,6)=qd6_0;
qd(tk,7)=qd7_0; qd(tk,8)=qd8_0; qd(tk,9)=qd9_0;
while (tt<=Tf)
   
   %% Compute Jacobian
   s1 = sin(qd(tk,1)); 
   s12 = sin(qd(tk,1)+qd(tk,2)); 
   s123 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3));
   s1234 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4));
   s12345 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5));
   s123456 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6));
   s1234567 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7));
   s12345678 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)+qd(tk,8));
   s123456789 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)+qd(tk,8)+qd(tk,9));
   c1 = cos(qd(tk,1)); 
   c12 = cos(qd(tk,1)+qd(tk,2)); 
   c123 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3));
   c1234 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4));
   c12345 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5));
   c123456 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6));
   c1234567 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7));
   c12345678 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)+qd(tk,8));
   c123456789 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6)+qd(tk,7)+qd(tk,8)+qd(tk,9));
    
    Jac1(1,9) = - s123456789;
    Jac1(1,8) = Jac1(1,9) - s12345678;
    Jac1(1,7) = Jac1(1,8) + c1234567;
    Jac1(1,6) = Jac1(1,7) + c123456;
    Jac1(1,5) = Jac1(1,6) - s12345;
    Jac1(1,4) = Jac1(1,5) - c1234;
    Jac1(1,3) = Jac1(1,4) - c123;
    Jac1(1,2) = Jac1(1,3) - s12;
    Jac1(1,1) = Jac1(1,2) - c1;
    Jac1(2,9) = c123456789;
    Jac1(2,8) = Jac1(2,9) + c12345678;
    Jac1(2,7) = Jac1(2,8) + s1234567;
    Jac1(2,6) = Jac1(2,7) + s123456;
    Jac1(2,5) = Jac1(2,6) + c12345;
    Jac1(2,4) = Jac1(2,5) - s1234;
    Jac1(2,3) = Jac1(2,4) - s123;
    Jac1(2,2) = Jac1(2,3) + c12;
    Jac1(2,1) = Jac1(2,2) - s1;

   %%Pseudo-inverse computation of Jacobian matrix Jac1
   Jac1_psinv = Jac1'*inv(Jac1*Jac1');
 
      %% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS *****
   %% ***** store successive positions for links 1, 2, and 3 (end-effector)
xd2(tk)=-s1;	%%% initial positions
yd2(tk)=c1;
xd3(tk)=-s1+c12;
yd3(tk)=c1+s12;
xd4(tk)=-s1+c12-s123;
yd4(tk)=c1+s12+c123;
xd5(tk)=-s1+c12-s123-s1234;
yd5(tk)=c1+s12+c123+c1234;
xd6(tk)=-s1+c12-s123-s1234+c12345;
yd6(tk)=c1+s12+c123+c1234+s12345;
xd7(tk)=-s1+c12-s123-s1234+c12345+s123456;
yd7(tk)=c1+s12+c123+c1234+s12345-c123456;
xd8(tk)=-s1+c12-s123-s1234+c12345+s123456+s1234567;
yd8(tk)=c1+s12+c123+c1234+s12345-c123456-c1234567;
xd9(tk)=-s1+c12-s123-s1234+c12345+s123456+s1234567+c12345678;
yd9(tk)=c1+s12+c123+c1234+s12345-c123456-c1234567+s12345678;
xdE(tk)=-s1+c12-s123-s1234+c12345+s123456+s1234567+c12345678+c123456789;
ydE(tk)=c1+s12+c123+c1234+s12345-c123456-c1234567+s12345678+s123456789;
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
   %Subtask 1
   task1=Jac1_psinv*[xd_(tk);yd_(tk)];
   
   %Subtask 2
   %obst(1,:) to katw, obst(2,:) to panw
   dist21 = pdist([xd2(tk) yd2(tk);obst(1,:)], 'euclidean');
   dist22 = pdist([xd2(tk) yd2(tk);obst(2,:)], 'euclidean');
   dist31 = pdist([xd3(tk) yd3(tk);obst(1,:)], 'euclidean');
   dist32 = pdist([xd3(tk) yd3(tk);obst(2,:)], 'euclidean');
   dist41 = pdist([xd4(tk) yd4(tk);obst(1,:)], 'euclidean');
   dist42 = pdist([xd4(tk) yd4(tk);obst(2,:)], 'euclidean');
   dist51 = pdist([xd5(tk) yd5(tk);obst(1,:)], 'euclidean');
   dist52 = pdist([xd5(tk) yd5(tk);obst(2,:)], 'euclidean');
   dist61 = pdist([xd6(tk) yd6(tk);obst(1,:)], 'euclidean');
   dist62 = pdist([xd6(tk) yd6(tk);obst(2,:)], 'euclidean');
   dist71 = pdist([xd7(tk) yd7(tk);obst(1,:)], 'euclidean');
   dist72 = pdist([xd7(tk) yd7(tk);obst(2,:)], 'euclidean');
   dist81 = pdist([xd8(tk) yd8(tk);obst(1,:)], 'euclidean');
   dist82 = pdist([xd8(tk) yd8(tk);obst(2,:)], 'euclidean');
   dist91 = pdist([xd9(tk) yd9(tk);obst(1,:)], 'euclidean');
   dist92 = pdist([xd9(tk) yd9(tk);obst(2,:)], 'euclidean');

   if dist21<dist22
       xo2 = 1;
       mindist2 = dist21;
   else
       xo2 = 2;
       mindist2 = dist22;
   end
   if dist31<dist32
       xo3 = 1;
       mindist3 = dist31;
   else
       xo3 = 2;
       mindist3 = dist32;
   end
   if dist41<dist42
       xo4 = 1;
       mindist4 = dist41;
   else
       xo4 = 2;
       mindist4 = dist42;
   end
   if dist51<dist52
       xo5 = 1;
       mindist5 = dist51;
   else
       xo5 = 2;
       mindist5 = dist52;
   end
   if dist61<dist62
       xo6 = 1;
       mindist6 = dist61;
   else
       xo6 = 2;
       mindist6 = dist62;
   end
   if dist71<dist72
       xo7 = 1;
       mindist7 = dist71;
   else
       xo7 = 2;
       mindist7 = dist72;
   end
   if dist81<dist82
       xo8 = 1;
       mindist8 = dist81;
   else
       xo8 = 2;
       mindist8 = dist82;
   end
   if dist91<dist92
       xo9 = 1;
       mindist9 = dist91;
   else
       xo9 = 2;
       mindist9 = dist92;
   end
   
   mindistall = min([mindist2 mindist3 mindist4 mindist5 mindist6 ...
       mindist7 mindist8 mindist9]);
   %edw to dVdq(1) einai gia ti 2i arthrwsi kai paei legontas
   if mindistall > 0.65
       dVdq(1:9) = 0;
   elseif mindistall == mindist2
       dVdq(1) = ((xd2(tk)-obst(xo2,1))*(- c1)+(yd2(tk)-obst(xo2,2))*(-s1))/mindist2;
       dVdq(2:9) = 0;
   elseif mindistall == mindist3
       dVdq(1) = ((xd3(tk)-obst(xo3,1))*(- c1 - s12)+(yd3(tk)-obst(xo3,2))*(-s1 + c12))/mindist3;
       dVdq(2) = ((xd3(tk)-obst(xo3,1))*(- s12)+(yd3(tk)-obst(xo3,2))*(c12))/mindist3;      
       dVdq(3:9) = 0;
   elseif mindistall == mindist4
       dVdq(1) = ((xd4(tk)-obst(xo4,1))*(- c1 - s12 - c123)+(yd4(tk)-obst(xo4,2))*(-s1 + c12 - s123))/mindist4;
       dVdq(2) = ((xd4(tk)-obst(xo4,1))*(- s12 - c123)+(yd4(tk)-obst(xo4,2))*(c12 - s123))/mindist4;
       dVdq(3) = ((xd4(tk)-obst(xo4,1))*(- c123)+(yd4(tk)-obst(xo4,2))*(- s123))/mindist4;
       dVdq(4:9) = 0;
   elseif mindistall == mindist5
       dVdq(1) = ((xd5(tk)-obst(xo5,1))*(- c1 - s12 - c123 - c1234)+(yd5(tk)-obst(xo5,2))*(-s1 + c12 - s123 - s1234))/mindist5;
       dVdq(2) = ((xd5(tk)-obst(xo5,1))*(- s12 - c123 - c1234)+(yd5(tk)-obst(xo5,2))*(c12 - s123 - s1234))/mindist5;
       dVdq(3) = ((xd5(tk)-obst(xo5,1))*(- c123 - c1234)+(yd5(tk)-obst(xo5,2))*(- s123 - s1234))/mindist5;
       dVdq(4) = ((xd5(tk)-obst(xo5,1))*(- c1234)+(yd5(tk)-obst(xo5,2))*(- s1234))/mindist5;
       dVdq(5:9) = 0;
   elseif mindistall == mindist6
       dVdq(1) = ((xd6(tk)-obst(xo6,1))*(- c1 - s12 - c123 - c1234 - s12345)+(yd6(tk)-obst(xo6,2))*(-s1 + c12 - s123 - s1234 + c12345))/mindist6;
       dVdq(2) = ((xd6(tk)-obst(xo6,1))*(- s12 - c123 - c1234 - s12345)+(yd6(tk)-obst(xo6,2))*(c12 - s123 - s1234 + c12345))/mindist6;
       dVdq(3) = ((xd6(tk)-obst(xo6,1))*(- c123 - c1234 - s12345)+(yd6(tk)-obst(xo6,2))*(- s123 - s1234 + c12345))/mindist6;
       dVdq(4) = ((xd6(tk)-obst(xo6,1))*(- c1234 - s12345)+(yd6(tk)-obst(xo6,2))*(- s1234 + c12345))/mindist6;
       dVdq(5) = ((xd6(tk)-obst(xo6,1))*(- s12345)+(yd6(tk)-obst(xo6,2))*(c12345))/mindist6;
       dVdq(6:9) = 0;
   elseif mindistall == mindist7
       dVdq(1) = ((xd7(tk)-obst(xo7,1))*(- c1 - s12 - c123 - c1234 - s12345 + c123456)+(yd7(tk)-obst(xo7,2))*(-s1 + c12 - s123 - s1234 + c12345 + s123456))/mindist7;
       dVdq(2) = ((xd7(tk)-obst(xo7,1))*(- s12 - c123 - c1234 - s12345 + c123456)+(yd7(tk)-obst(xo7,2))*(c12 - s123 - s1234 + c12345 + s123456))/mindist7;
       dVdq(3) = ((xd7(tk)-obst(xo7,1))*(- c123 - c1234 - s12345 + c123456)+(yd7(tk)-obst(xo7,2))*(- s123 - s1234 + c12345 + s123456))/mindist7;
       dVdq(4) = ((xd7(tk)-obst(xo7,1))*(- c1234 - s12345 + c123456)+(yd7(tk)-obst(xo7,2))*(- s1234 + c12345 + s123456))/mindist7;
       dVdq(5) = ((xd7(tk)-obst(xo7,1))*(- s12345 + c123456)+(yd7(tk)-obst(xo7,2))*(c12345 + s123456))/mindist7;
       dVdq(6) = ((xd7(tk)-obst(xo7,1))*(c123456)+(yd7(tk)-obst(xo7,2))*(s123456))/mindist7;
       dVdq(7:9) = 0;
   elseif mindistall == mindist8
       dVdq(1) = ((xd8(tk)-obst(xo8,1))*(- c1 - s12 - c123 - c1234 - s12345 + c123456 + c1234567)+(yd8(tk)-obst(xo8,2))*(-s1 + c12 - s123 - s1234 + c12345 + s123456 + s1234567))/mindist8;
       dVdq(2) = ((xd8(tk)-obst(xo8,1))*(- s12 - c123 - c1234 - s12345 + c123456 + c1234567)+(yd8(tk)-obst(xo8,2))*(c12 - s123 - s1234 + c12345 + s123456 + s1234567))/mindist8;
       dVdq(3) = ((xd8(tk)-obst(xo8,1))*(- c123 - c1234 - s12345 + c123456 + c1234567)+(yd8(tk)-obst(xo8,2))*(- s123 - s1234 + c12345 + s123456 + s1234567))/mindist8;
       dVdq(4) = ((xd8(tk)-obst(xo8,1))*(- c1234 - s12345 + c123456 + c1234567)+(yd8(tk)-obst(xo8,2))*(- s1234 + c12345 + s123456 + s1234567))/mindist8;
       dVdq(5) = ((xd8(tk)-obst(xo8,1))*(- s12345 + c123456 + c1234567)+(yd8(tk)-obst(xo8,2))*(c12345 + s123456 + s1234567))/mindist8;
       dVdq(6) = ((xd8(tk)-obst(xo8,1))*(c123456 + c1234567)+(yd8(tk)-obst(xo8,2))*(s123456 + s1234567))/mindist8;
       dVdq(7) = ((xd8(tk)-obst(xo8,1))*(c1234567)+(yd8(tk)-obst(xo8,2))*(s1234567))/mindist8;
       dVdq(8:9) = 0;
   elseif mindistall == mindist9
       dVdq(1) = ((xd9(tk)-obst(xo9,1))*(- c1 - s12 - c123 - c1234 - s12345 + c123456 + c1234567 - s12345678)+(yd9(tk)-obst(xo9,2))*(-s1 + c12 - s123 - s1234 + c12345 + s123456 + s1234567 + c12345678))/mindist9;
       dVdq(2) = ((xd9(tk)-obst(xo9,1))*(- s12 - c123 - c1234 - s12345 + c123456 + c1234567 - s12345678)+(yd9(tk)-obst(xo9,2))*(c12 - s123 - s1234 + c12345 + s123456 + s1234567 + c12345678))/mindist9;
       dVdq(3) = ((xd9(tk)-obst(xo9,1))*(- c123 - c1234 - s12345 + c123456 + c1234567 - s12345678)+(yd9(tk)-obst(xo9,2))*(- s123 - s1234 + c12345 + s123456 + s1234567 + c12345678))/mindist9;
       dVdq(4) = ((xd9(tk)-obst(xo9,1))*(- c1234 - s12345 + c123456 + c1234567 - s12345678)+(yd9(tk)-obst(xo9,2))*(- s1234 + c12345 + s123456 + s1234567 + c12345678))/mindist9;
       dVdq(5) = ((xd9(tk)-obst(xo9,1))*(- s12345 + c123456 + c1234567 - s12345678)+(yd9(tk)-obst(xo9,2))*(c12345 + s123456 + s1234567 + c12345678))/mindist9;
       dVdq(6) = ((xd9(tk)-obst(xo9,1))*(c123456 + c1234567 - s12345678)+(yd9(tk)-obst(xo9,2))*(s123456 + s1234567 + c12345678))/mindist9;
       dVdq(7) = ((xd9(tk)-obst(xo9,1))*(c1234567 - s12345678)+(yd9(tk)-obst(xo9,2))*(s1234567 + c12345678))/mindist9;
       dVdq(8) = ((xd9(tk)-obst(xo9,1))*(- s12345678)+(yd9(tk)-obst(xo9,2))*(c12345678))/mindist9;
       dVdq(9) = 0;
   end 
    
   H2=eye(9)*kc;
   %edw thelei prosthiki * kati 
   task2 = (eye(9)-Jac1_psinv*Jac1)*H2*dVdq';
   
   %angular velocity
   qd_(tk,:) = task1'+task2';
   


   
   %% numerical integration --> kinematic simulation
   qd(tk+1,1) = qd(tk,1) + dt*qd_(tk,1);
   qd(tk+1,2) = qd(tk,2) + dt*qd_(tk,2);
   qd(tk+1,3) = qd(tk,3) + dt*qd_(tk,3);
   qd(tk+1,4) = qd(tk,4) + dt*qd_(tk,4);
   qd(tk+1,5) = qd(tk,5) + dt*qd_(tk,5);
   qd(tk+1,6) = qd(tk,6) + dt*qd_(tk,6);
   qd(tk+1,7) = qd(tk,7) + dt*qd_(tk,7);
   qd(tk+1,8) = qd(tk,8) + dt*qd_(tk,8);
   qd(tk+1,9) = qd(tk,9) + dt*qd_(tk,9);
   
   
  	%% ***** PLOT MOTION - ANIMATE THE ROBOT SIMULATION *****
   %% (also, display moving box)
   if (mod(tk,dtk)==0),
      cla						%% clear plot at each time step (also possible to consider double-buffering?...)
      plot(xd, yd, 'm:');  %% replot desired trajectory (if given)
      
rectangle('Position',[xobst,yobst+1.5,0.5,0.5],...
  'Curvature',[1,1], 'FaceColor','r')
rectangle('Position',[xobst,yobst,0.5,0.5],...
  'Curvature',[1,1], 'FaceColor','r')

		plot([0], [0], 'o');
  		plot([0, xd2(tk)], [0, yd2(tk)],'r');
	  	plot([xd2(tk)], [yd2(tk)], '*');
  		plot([xd2(tk), xd3(tk)], [yd2(tk), yd3(tk)],'m');
	  	plot([xd3(tk)], [yd3(tk)], '*');
  		plot([xd3(tk), xd4(tk)], [yd3(tk), yd4(tk)],'b');
        plot([xd4(tk)], [yd4(tk)], '*');
  		plot([xd4(tk), xd5(tk)], [yd4(tk), yd5(tk)],'m');
        plot([xd5(tk)], [yd5(tk)], '*');
  		plot([xd5(tk), xd6(tk)], [yd5(tk), yd6(tk)],'b');
        plot([xd6(tk)], [yd6(tk)], '*');
  		plot([xd6(tk), xd7(tk)], [yd6(tk), yd7(tk)],'m');
        plot([xd7(tk)], [yd7(tk)], '*');
  		plot([xd7(tk), xd8(tk)], [yd7(tk), yd8(tk)],'b');
        plot([xd8(tk)], [yd8(tk)], '*');
  		plot([xd8(tk), xd9(tk)], [yd8(tk), yd9(tk)],'m');
        plot([xd9(tk)], [yd9(tk)], '*');
        plot([xd9(tk), xdE(tk)], [yd9(tk), ydE(tk)],'m');
	  	plot([xd9(tk)], [yd9(tk)], 'y*');
     	plot([xd9(tk)], [yd9(tk)], 'g+');
       
      pause(0.3); %% pause motion to view successive robot configurations
   end
    


   
   tk=tk+1;	 %step increment, and
   tt=tt+dt; %time increment (for the simulation)
end




%% *** SAVE and PLOT output data ***
%%** use functions plot(...)
%save;  %% --> save data to 'matlab.mat' file

fig1 = figure;

subplot(2,2,1);
plot(t,xd);
ylabel('xd (cm)');
xlabel('time t (sec)');


subplot(2,2,2);
plot(t,yd);
ylabel('yd (cm)');
xlabel('time t (sec)');


subplot(2,2,3);
plot(t,qd(:,1));
ylabel('qd1 (rad)');
xlabel('time t (sec)');


subplot(2,2,4);
plot(t,qd(:,2));
ylabel('qd2 (rad)');
xlabel('time t (sec)');






















