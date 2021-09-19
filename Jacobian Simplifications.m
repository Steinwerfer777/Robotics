%C. Michael Collins
syms x3 x4;

x1 = 0;
x2 = 20;
d1 = 0.5;
d3 = 0.8;
a = 0.2;

%x1 = Joint Variable Theta 1
%x2 = Joint Variable Theta 2
%d3 is Joint Variable d

%For hard values
%d = 1;
%a2 = 0.6;
%a3 = 1.2;
%m1 = 10;
%m2 = 5;
%m3 = 8;

%Step 1 Basic Extractions

R01 = [cosd(x1) 0 sind(x1);sind(x1) 0 -cosd(x1); 0 1 0];
R12 = [cosd(x2) 0 -sind(x2);sind(x2) 0 cosd(x2);0 -1 0];
R23 = [1 0 0;0 0 -1;0 1 0];
R34 = [cosd(x4) -sind(x4) 0;sind(x4) cosd(x4) 0;0 0 1]
R02 = R01*R12;
R03 = R01*R12*R23;
R04 = R01*R12*R23*R34;
r01 = [0;0;d1];
r12 = [0;0;0];
r23 = [0; 0; d3];
r34 = a*[cosd(x4);sind(x4);0];

%Step 2 Position Vectors for Jacobian Standard

z0 = [0;0;1];
z1 = R01*z0;
z2 = R02*z0;
z3 = R03*z0; 

P34 = simplify(R03*r34);
P24 = simplify(R02*r23 + P34); 
P14 = simplify(R01*r12 + P24);
P04 = simplify(r01 + P14); 

%Step 3 Jacobian for 2 Revolute Joints & a prismatic

Jv1 = cross(z0, P04, 1);
Jw1 = z0;
Jv2 = cross(z1, P14,1);
Jw2 = z1;
Jv3 = z2;
Jw3 = [0;0;0];
Jv4 = cross(z3, P34, 1);
Jw4 = z3;

%Standard Jacobian 
J = simplify([Jv1 Jv2 Jv3 Jv4;Jw1 Jw2 Jw3 Jw4]); 

