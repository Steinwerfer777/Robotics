%Robotics Assignment 4 simplification equations
%C. Michael Collins
syms x1 x2 x3 d3 d a2 a3 m1 m2 m3 l; 

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

R01 = [cos(x1) 0 sin(x1);sin(x1) 0 -cos(x1);0 1 0];
R12 = [cos(x2) -sin(x2) 0;sin(x2) cos(x2) 0;0 0 1];
R23 = [cos(x3) -sin(x3) 0;sin(x3) cos(x3) 0;0 0 1];
R02 = R01*R12;
R03 = R01*R12*R23;
r01 = [0;0;l];
r12 = [l*cos(x2);l*sin(x2);0];
r23 = 0.6*r12;


%Step 2 Inertia Matrix from Coordinate Frames

I11 = ((m1*d^2)/12)*[1 0 0;0 0 0;0 0 1];
I22 = ((m2*a2^2)/12)*[1 0 0;0 1 0;0 0 0];
I33 = ((m3*a3^2)/12)*[1 0 0;0 1 0;0 0 0];

%Step 3 Inertia Moments

I1 = R01*I11*transpose(R01);
I2 = R02*I22*transpose(R02);
I3 = R03*I33*transpose(R03);

%Step 4: Position Vectors for Dynamic Matrix 

r1_1c1 = [0;-d/2;0];
r2_2c2 = [0;0;a2/2];
r3_3c3 = [0;0;-a3/2];
P0c1 = R01*r1_1c1 + r01;
P1c2 = R02*r2_2c2 + R01*r12;
P0c2 = simplify(r01 + P1c2);
P2c3 = R03*r3_3c3 + R02*r23;
P1c3 = simplify(R01*r12 + P2c3);
P0c3 = simplify(r01 + P1c3);

%Step 5 Jacobian Links

z0 = [0;0;1];
z1 = R01*z0;
z2 = R02*z0;
zero = [0;0;0];

%i = j = 1
Jv1 = simplify([cross(z0, P0c1, 1) zero zero]);
Jw1 = [z0 zero zero];
%i = 2; j = 1 then 2
Jw21 = z0;
Jw22 = z1;
Jw2 = [Jw21 Jw22 zero];
Jv21 = cross(z0, P0c2, 1);
Jv22 = cross(z1, P1c2,1);
Jv2 = simplify([Jv21 Jv22 zero]);
%i = 3; j = 1, 2, then 3
Jv31 = cross(z0, P0c3,1);
Jw31 = z0;
Jv32 = cross(z1, P1c3,1);
Jw32 = z1;
Jv33 = z2; 
Jv3 = simplify([Jv31 Jv32 Jv33]);
Jw33 = zero;
Jw3 = [Jw31 Jw32 Jw33];

%Step 6 Manipulator Equation

M_1 = transpose(Jv1)*m1*Jv1+transpose(Jw1)*I1*Jw1+transpose(Jv2)*m2*Jv2+transpose(Jw2)*I2*Jw2+transpose(Jv3)*m3*Jv3+transpose(Jw3)*I3*Jw3;
M = simplify(M_1);
M11 = simplify(M(1,1));
M12 = simplify(M(1,2));
M13 = simplify(M(1,3));
M21 = simplify(M(2,1));
M22 = simplify(M(2,2));
M23 = simplify(M(2,3));
M31 = simplify(M(3,1));
M32 = simplify(M(3,2));
M33 = simplify(M(3,3));

%Step 7 Non-Linear

syms t dt1 dt2 dd3;

N1 = diff(M(1,1),x2)*dt1*dt2 + diff(M(1,1),d3)*dt1*dd3; 
N2 = diff(M(2,2), d3)*dt2*dd3;
N3 = 0;
N_1 = [N1;N2;N3];
N = simplify(N_1);

%Step 8 Gravity

g = 10; %gravity constant rounded up

gm = [0;0;-g];
gT = transpose(gm);
G1 = m1*gT*Jv1(:,1)+m2*gT*Jv2(:,1)+m3*gT*Jv3(:,1);
G2 = m1*gT*Jv1(:,2)+m2*gT*Jv2(:,2)+m3*gT*Jv3(:,2);
G3 = m1*gT*Jv1(:,3)+m2*gT*Jv2(:,3)+m3*gT*Jv3(:,3);
G_1 = [-G1;-G2;-G3];
G = simplify(G_1);

%Step 9 Dynamic Equation Overall
syms ddt1 ddt2 ddd3;
ddq = [ddt1; ddt2; ddd3];
%Torque
T = M*ddq+N+G;