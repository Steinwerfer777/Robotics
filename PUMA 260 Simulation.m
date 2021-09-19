Simulation of PUMA 260 

%Name: C. Michael Collins
%Date Constructed: July 4th, 2020 [INDEPENDENCE DAY!]

%L = Link(theta(rad), d (cm), a (cm), alpha (rad))
%PUMA has 6DOF, so 6 links

L(1) = Link([0 35 0 -pi/2]);
L(2) = Link([0 -5 20 0]);
L(3) = Link([0 0 0 -pi/2]);
L(4) = Link([0 20 0 pi/2]);
L(5) = Link([0 0 0 -pi/2]);
L(6) = Link([0 5 0 0]);

%No offset required for home position
%Safe Planar3d in simulation and assign
 
planar3d = SerialLink(L);
planar3d.name = 'PUMA PRIDE!';

qz = [0 0 0 0 0 0]; %angles of theta for each link in home position

%Generated plot for home position

%Note that the simulation plots do not save for individual figures, so whichever simulation you want to view, comment out the other two.

figure (1)
planar3d.plot(qz);

%Forward Kinematics

q2 = [pi/3 pi/24 -pi/4 pi/4 pi/4 pi/4]; %Case 2 in Rads
q3 = [pi/24 0 pi/3 pi/4 pi -pi/2]; %Case 3 in Rads

T_home = planar3d.fkine(qz); %Matrix for Home position
T_2 = planar3d.fkine(q2); %Matrix for Case 2
T_3 = planar3d.fkine(q3); %Matrix for Case 3 

%Generated plots for Cases 2 & 3
%planar3d.plot(q2); %Uncomment then comment other plots
%planar3d.plot(q3); %Uncomment then comment other plots

%X-Position versus Time plot generated from Simulink values

%NEXT FIGURE COMING

%figure (2)
%plot(X.time, X.signals.values)
%xlabel('time (s)')
%ylabel('X-axis (cm)')
%title("X-axis Coordinate Motion Over Time")

%Y-Position versus Time Plot generated from Simulink values
%figure (3)
%plot(Y.time, Y.signals.values)
%xlabel('time(s)')
%ylabel('Y-axis (cm)')
%title("Y-axis Coordinate Motion Over Time")

%Inverse Kinematics
%Using code provided for calculating Inverse Kinematics

q2_i = planar3d.ikine(T_2); %to get one of the 32 solutions
T_i = planar3d.fkine(q2_i); %should get back T_2

%Parameter of links
d1=35; %a
d2=-5; %b
d4=20; %d
d6=5; %e
a2=20; % c
a3=0; % 0 for PUMA 260

%Define the hand coordinates with respect to base 0T6
% Using 0T6 for [60 15 -45 45 45 45]
 OT6 = [ 0.8333 -0.3639 -0.4161 18.2516;
        -0.2638 -0.9233 0.2793 26.6127;
        -0.4858 -0.1229 -0.8654 12.1956;
          0         0         0    1.0000];

%Extract unit vectors
%Position is defined by the vector q
u=OT6(1:3,1);
v=OT6(1:3,2);
w=OT6(1:3,3);
q=OT6(1:3,4);
%-------------------------------------------------
%Find the wist centre position p
p=q-d6.*w;
theta1_1=(-p(1)+sqrt(p(1)^2+p(2)^2-d2^2))/(d2+p(2));
theta1_2=(-p(1)-sqrt(p(1)^2+p(2)^2-d2^2))/(d2+p(2));
%-------------------------------------------------
%-----Theta 1 (four solutions)---------------------
t1=zeros(4,1);
t1(1)=2*atan(theta1_1);
t1(2)=t1(1)+pi;
t1(3)=2*atan(theta1_2);
t1(4)=t1(3)+pi;
%-------------------------------------------------

%-----Theta 3 (four solutions)-------------------- 
k1=-2*a2*d4;
k2=2*a2*a3;
k3=p(1)^2+p(2)^2+(d1-p(3))^2-(a2^2+a3^2+d2^2+d4^2);
theta3_1=(k1+sqrt(k1^2+k2^2-k3^2))/(k2+k3);
theta3_2=(k1-sqrt(k1^2+k2^2-k3^2))/(k2+k3);

t3=zeros(4,1);
t3(1)=2*atan(theta3_1);
t3(2)=t3(1)+pi;
t3(3)=2*atan(theta3_2);
t3(4)=t3(3)+pi;

%---------------------------------------------------------
%-----Theta 2 (16 solutions)
%-find theta 2 from 4 solutions theta 1 and 4 solutions theta 3 
kk=1;
t2=zeros(16,1);
tt1=zeros(16,1);
tt3=zeros(16,1);

for i=1:4
for ii=1:4
    al1=-d4*sin(t3(i))+a3*cos(t3(i))+a2;
    al2=d4*cos(t3(i))+a3*sin(t3(i));
    
    bet1=-d4*cos(t3(i))-a3*sin(t3(i));
    bet2=-d4*sin(t3(i))+a3*cos(t3(i))+a2;
    
    gam1=p(1)*cos(t1(ii))+p(2)*sin(t1(ii));
    gam2=-p(3)+d1;
    
    sint2=(al2*gam1-al1*gam2)/(al2*bet1-al1*bet2);
    cost2=(bet2*gam1-bet1*gam2)/(bet2*al1-bet1*al2);
    
    t2(kk)=atan2(sint2,cost2);
    tt3(kk)=t3(i);
    tt1(kk)=t1(ii);
    kk=kk+1;  
end
end
t1=tt1;
t3=tt3;
theta1=t1.*(180/pi);
theta2=t2.*(180/pi);
theta3=t3.*(180/pi);
%--------------------------------------------------
%-----Theta 5 (two solutions)

t5=zeros(32,1);
r33=-(w(1).*cos(t1).*sin(t2+t3)+w(2).*sin(t1).*sin(t2+t3)+w(3).*cos(t2+t3));

t5_1=acos(r33);
t5_2=2*pi-t5_1;

t5=[t5_1;t5_2];
theta5=t5.*(180/pi);

t1=[t1;t1];
t2=[t2;t2];
t3=[t3;t3];

%------------------------------------------
%-----Theta 4-----
r13=(w(1).*cos(t1).*cos(t2+t3)+w(2).*sin(t1).*cos(t2+t3)-w(3).*sin(t2+t3));
r23=w(1).*sin(t1)-w(2).*cos(t1);
r31=-(u(1).*cos(t1).*sin(t2+t3)+u(2).*sin(t1).*sin(t2+t3)+u(3).*cos(t2+t3));
r32=-(v(1).*cos(t1).*sin(t2+t3)+v(2).*sin(t1).*sin(t2+t3)+v(3).*cos(t2+t3));

t4=atan2(-r23./sin(t5),-r13./sin(t5));
theta4=t4.*(180/pi);

%--------------------------------------------------------------------------
%-----Theta 6-----
t6=atan2(-r32./sin(t5),r31./sin(t5));
theta6=t6.*(180/pi);

%--------------------------------------------------------------------------
%-----Results
theta1=[theta1;theta1];
theta2=[theta2;theta2];
theta3=[theta3;theta3];
theta=[round(theta1),round(theta2),round(theta3),round(theta4),round(theta5),round(theta6)];
for(j=1:32)
    for(jj=1:6)
        if(theta(j,jj)>180);
            theta(j,jj)=theta(j,jj)-360;
        end
    end
end
theta
%Solution to the nearest degree
theta_r=round(theta);

%Final check for viable solutions based on forward kinematics 
%Viable angles after considering safety and limitations
q1 = [-120 127 -45 -150 82 75];
q2 = [-138 127 -45 -151 80 60];
q3 = [60 53 -135 30 82 75];
q_4 = [-120 173 -135 -135 45 45];
q_5 = [-138 173 -135 -142 52 34];
q_6 = [42 53 -135 29 90 60];
q_7 = [60 -37 45 106 31 -29];
q_8 = [60 7 -45 -135 -45 -135];
q_9 = [42 7 -45 -142 -52 -146];
q_10 = [60 53 -135 -150 -82 -105];
q_11 = [42 53 -135 -151 -90 -120];
q_12 = [60 -37 45 -74 -31 151];
q_13 = [42 -37 45 -91 -29 151];

%Matrix solution should match Case 2's Matrix 
%Add in whichever theta value above 
T_t = planar3d.fkine(deg2rad(q_13));
