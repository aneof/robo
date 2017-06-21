%% *** Robot (kinematic) model parameters *** 
clear all; 
close all; 
          %diastaseis se cm
l(1)=10.0;   % = l0
l(2) = 10.0;  %% = l1
l(3) = 8.0;  %= l2
l(4)=0.0; % l3=0,symfwna me ta dedomena tou problhmatos
l(5)=20.0;
l(6)=15.0;
%% *** sampling period *** 
%% *** for the robot motion, kinematic simulation: 
dt = 0.001; %dt = 0.001; i.e. 1 msec)   

%% *** Create (or load from file) reference signals *** 
%% *** DESIRED MOTION PROFILE - TASK SPACE *** 
Tf=10.0; 	% 10sec duration of motion 
t=0:dt:Tf;  

%xd0,td0,yd1: initial/final end-point position --> desired task-space trajectory  
xd0 = 12.0;	
xd1 =  12.0;  %xa=xb
yd0 = 10.00; 
yd1 = 0.0;  
zd0 = 15.0;
zd1 = 15.0;    %za=zb

% Example of desired trajectory : linear segment (x0,y0)-->(x1,y1); Time duration: Tf; 
disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...'); %% 
disp(' ');   
xd(1) = xd0; 
yd(1) = yd0; 
zd(1) = zd0;
lambda_x = (xd1-xd0)/Tf; 
%lambda_y = (yd1-yd0)/Tf; 
lambda_z = (zd1-zd0)/Tf;
kmax=Tf/dt + 1; 
for k=2:kmax;    
   xd(k) = xd(k-1) + lambda_x*dt;    
   %yd(k) = yd(k-1) + lambda_y*dt; 
   zd(k) = zd(k-1) + lambda_z*dt;
end  
 
 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% ****** KINEMATIC SIMULATION - Main loop ****** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %%  

%% ***** INVERSE KINEMATICS  -->  DESIRED MOTION - JOINT SPACE ***** 
%% compute the reference joint-motion vectors: 
%% {qd(k,i), i=1,...,n (num of degrees of freedom), with k=1,..., kmax,} 
%% se antistoixia methodou kai simbolismwn me ti lisi tou merous A: 
Pexstar = xd - l(2);
Pezstar = zd + l(1);
k(1:10001) = 1;
K = sqrt(Pexstar(:).^2 + Pezstar(:).^2 - l(3)^2)';  
qd(:,1) = atan2(Pezstar , Pexstar) - atan2(K , l(3)*k);

%sxediasmos troxias mesw poliwnimikis sinartisis paremboplis (diafaneies)
%4A_robotics-I-shmmy-control-1.pdf


v0 = 0;   %arxiki kai teliki taxitita 0
vf = 0; 

a0 = yd0; 
a1= v0;
a2=(3/Tf^2)*(yd1-yd0)-(2/Tf)*a1-(1/Tf)*vf;
a3=(-2/Tf^3)*(yd1-yd0)+(1/10^2)*(v0+vf);

%Pey(t)=a0+a1*t+a2*t^2+a3*t^3
yd=a0+a1*t(:)+a2*t(:).^2+a3*t(:).^3;
ydA=yd'; %gia simfwnia diastasewn argotera
%euresi ipoloipwn qi
s1=sin(qd(:,1)); 
c1=cos(qd(:,1)); 
Kstar = -K - l(4);
Kstar = Kstar';
qd(:,3) = acos(( Kstar(:).^2 + yd(:).^2 -l(5)^2 -l(6)^2)./(2*l(5)*l(6)));
s3=sin(qd(:,3));
c3=cos(qd(:,3));
qd(:,2) = atan2( yd , Kstar ) - atan2(l(6)*s3,l(5) + l(6)*c3 );

s2=sin(qd(:,2));
c2=cos(qd(:,2));
s23 = sin(qd(:,2)+qd(:,3));
c23 = cos(qd(:,2)+qd(:,3));
   
%% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS ***** 
%simeio 0
xd1(1:kmax) = 0;   
yd1(1:kmax) = 0; 
zd1(1:kmax) = -l(1); 
xd2(1:kmax) = l(2); %simeio 0'
yd2(1:kmax) = 0;
zd2(1:kmax) = -l(1); 
xd3(1:kmax) = l(2)+l(3).*c1; %simeio 1  
yd3(1:kmax) = 0; 
zd3(1:kmax) = -l(1)+l(3).*s1;
xd4(1:kmax) = l(2)+ l(3).*c1+(l(4)+l(5)*c2).*s1; %simeio 2
yd4(1:kmax) = l(5).*s2;
zd4(1:kmax) = -l(1)+l(3).*c1-(l(4)+l(5)*c2).*s1;
%kai sintetagmenes telikou simeiou
xd5 = l(2)+l(3)*c1+s1.*(l(5)*c2+l(6)*c23);
yd5 = l(5)*s2+l(6)*s23;
zd5 = -l(1)+l(3)*s1-(l(4)+l(5)*c2+l(6)*c23).*c1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

%% *** SAVE and PLOT output data *** %%** use functions plot(...)  
%save;  %% --> save data to 'matlab.mat' file   

figure(1); 
subplot(2,2,1); 
plot(t,xd); 
ylabel('Pex (cm)'); 
xlabel('time (sec)');  

subplot(2,2,2); 
plot(t,yd); 
ylabel('Pey (cm)'); 
xlabel('time (sec)');  

subplot(2,2,3); 
plot(t,zd); 
ylabel('Pez (cm)'); 
xlabel('time (sec)');  

figure(2);  
subplot(2,2,1); 
plot(t,qd(:,1)); 
ylabel('qd1 (rad)'); 
xlabel('time (sec)');  

subplot(2,2,2); 
plot(t,qd(:,2)); 
ylabel('qd2 (rad)'); 
xlabel('time (sec)');  

subplot(2,2,3); 
plot(t,qd(:,3)); 
ylabel('qd3 (rad)'); 
xlabel('time (sec)');

 %taxitites telikou simeiou
Vex = diff(xd);
Vex = [Vex, 0]; %prosthesi midenikou gia na einai isomegethi
     
Vey = diff(ydA);
Vey = [Vey, 0];

Vez = diff(zd);
Vez = [Vez, 0];


figure(3);
subplot(2,2,1); 
plot(t,Vex); 
ylabel('Vex (cm/sec)'); 
xlabel('time (sec)');  

subplot(2,2,2); 
plot(t,Vey); 
ylabel('Vey (cm/sec)'); 
xlabel('time t (sec)');  

subplot(2,2,3); 
plot(t,Vez); 
ylabel('Vez (cm/sec)'); 
xlabel('time (sec)');  


figure(4); % Iakwbiani kai antistrofi Iakwbiani, wste na broume tis taxitites twn qi

for k=1:kmax;
    J11 = -l(3)*s1(k)+(l(4)+l(5)*c2(k)+l(6)*c23(k)).*c1(k);
    J21 = 0;
    J31 = l(3)*c1(k)+(l(4)+l(5)*c2(k)+l(6)*c23(k)).*s1(k);    
    J12 = -(l(5)*s2(k)+l(6)*s23(k)).*s1(k);
    J22 = l(5)*c2(k)+l(6)*c23(k);
    J32 = (l(5)*s2(k)+l(6)*s23(k)).*c1(k);    
    J13 = -l(6)*s23(k).*s1(k);
    J23 = l(6)*c23(k);
    J33 = l(6)*s23(k).*c1(k);
    
    J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];
    invJ = inv(J);
    
    Vq1(k) = invJ(1,:)*[ Vex(k);Vey(k);Vez(k) ];
    Vq2(k) = invJ(2,:)*[ Vex(k);Vey(k);Vez(k) ];
    Vq3(k) = invJ(3,:)*[ Vex(k);Vey(k);Vez(k) ];
end

subplot(2,2,1); 
plot(t,Vq1); 
ylabel('Vq1 (rad/sec)'); 
xlabel('time (sec)');

subplot(2,2,2); 
plot(t,Vq2); 
ylabel('Vq2 (rad/sec)'); 
xlabel('time (sec)');

subplot(2,2,3); 
plot(t,Vq3); 
ylabel('Vq3 (rad/sec)'); 
xlabel('time (sec)');  

%%*** stick diagram --> animate robot motion ... (**optional**) 
%% within a for (or while) loop, use periodic plot(...) functions to draw the geometry (current pos)  
%% of the robot, and thus animate its motion ...  

figure(5); 
axis([-25 25 -20 20 -20 20]) %%set xyz plot axes (caution: square axes, i.e. dx=dy) 
axis on
grid on
hold on 
xlabel('x (cm)'); 
ylabel('y (cm)'); 
zlabel('z (cm)'); 
plot3(xd,yd,zd,'rs'); 
dtk=1000; %% plot robot position every dtk samples, to animate its motion 
plot3([0],[0],[0],'o'); 
for tk=1:dtk:kmax,    %%% 	
   pause(0.1);	%% pause motion to view successive robot configurations    
   
   plot3([0,xd1(tk)],[0,yd1(tk)],[0,zd1(tk)]);					
   plot3([xd1(tk)],[yd1(tk)],[zd1(tk)],'o');       
   plot3([xd1(tk),xd2(tk)],[yd1(tk),yd2(tk)],[zd1(tk),zd2(tk)]);	
   plot3([xd2(tk)],[yd2(tk)],[zd2(tk)],'o');     
   plot3([xd2(tk),xd3(tk)],[yd2(tk),yd3(tk)],[zd2(tk),zd3(tk)]);	
   plot3([xd3(tk)],[yd3(tk)],[zd3(tk)],'o');   
   plot3([xd3(tk),xd4(tk)],[yd3(tk),yd4(tk)],[zd3(tk),zd4(tk)]);
   plot3([xd4(tk)],[yd4(tk)],[zd4(tk)],'o');     
   plot3([xd4(tk),xd5(tk)],[yd4(tk),yd5(tk)],[zd4(tk),zd5(tk)]);
   plot3([xd5(tk)],[yd5(tk)],[zd5(tk)],'g+');
end

%% deutero plot apo diaforetiki pleura 
figure(6); 
axis([-25 25 -20 20 -20 20]) %%set xyz plot axes (caution: square axes, i.e. dx=dy) 
axis on
grid on
hold on 
xlabel('x (cm)'); 
ylabel('y (cm)'); 
zlabel('z (cm)'); 
plot3(yd,xd,zd,'rs'); 
dtk=1000; %% plot robot position every dtk samples, to animate its motion 
plot3([0],[0],[0],'o'); 
for tk=1:dtk:kmax,    %%% 	
   pause(0.1);	%% pause motion to view successive robot configurations    
   
   plot3([0,yd1(tk)],[0,xd1(tk)],[0,zd1(tk)]);					
   plot3([yd1(tk)],[xd1(tk)],[zd1(tk)],'o');       
   plot3([yd1(tk),yd2(tk)],[xd1(tk),xd2(tk)],[zd1(tk),zd2(tk)]);	
   plot3([yd2(tk)],[xd2(tk)],[zd2(tk)],'o');     
   plot3([yd2(tk),yd3(tk)],[xd2(tk),xd3(tk)],[zd2(tk),zd3(tk)]);	
   plot3([yd3(tk)],[xd3(tk)],[zd3(tk)],'o');   
   plot3([yd3(tk),yd4(tk)],[xd3(tk),xd4(tk)],[zd3(tk),zd4(tk)]);
   plot3([yd4(tk)],[xd4(tk)],[zd4(tk)],'o');     
   plot3([yd4(tk),yd5(tk)],[xd4(tk),xd5(tk)],[zd4(tk),zd5(tk)]);
   plot3([yd5(tk)],[xd5(tk)],[zd5(tk)],'g+');
end