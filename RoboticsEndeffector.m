syms a1 a2 m3 g fx fy fz x1 x2 x3 l;

Jv = [-(a1*sin(x1)+sin(x2+x1)) -a2*sin(x1+x2) 0;(a1*cos(x1)+a2*cos(x1+x2)) a2*cos(x1+x2) 0;0 0 -1];
Jw = [0 0 0;0 0 0;1 1 0];
J = [Jv;Jw];
F = [fx;fy;fz;0;0;0];
G = [0;0;-m3*g];

%Q = J^T*F+G
T = transpose(J)*F + G;

%Rotations
a1 = pi/9;
a2 = 2.443;
a3 = -pi/3;

R1 = [cos(a1) -sin(a1) 0;sin(a1) cos(a1) 0;0 0 1];
R2 = [cos(a2) 0 sin(a2); 0 1 0;-sin(a2) 0 cos(a2)];
R3 = [cos(a3) -sin(a3) 0;sin(a3) cos(a3) 0;0 0 1];
R = R1*R2*R3;

b1 = atan2(-0.557/cos(0.3274), -0.7657/cos(0.3274));
b2 = atan2(-0.9447/cos(0.3274), -0.0636/cos(0.3274));


T1 = [cos(x1) 0 sin(x1) 0;sin(x1) 0 cos(x1) 0;0 1 0 l;0 0 0 1];
T2 = [cos(x2) -sin(x2) 0 l*cos(x2);sin(x2) cos(x2) 0 l*sin(x2); 0 0 1 0;0 0 0 1];
T3 = [cos(x3) -sin(x3) 0 0.6*l*cos(x3);sin(x3) cos(x3) 0 0.6*l*sin(x3);0 0 1 0;0 0 0 1];
T4 = T1*T2*T3;
T5 = inv(T4);

c1 = atan2(-0.1736, 0.9848);

