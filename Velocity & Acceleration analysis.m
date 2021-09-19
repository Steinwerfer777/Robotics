%Code for linear velocity & acceleration

syms l1 l2 l3 x1 x2 x3 dx1 dx2 dx3;

R01 = [cos(x1) 0 sin(x1);sin(x1) 0 -cos(x1);0 1 0];
R12 = [cos(x2) -sin(x2) 0;sin(x2) cos(x2) 0;0 0 1];
R23 = [cos(x3) -sin(x3) 0;sin(x3) cos(x3) 0;0 0 1];
R10 = simplify(inv(R01));
R21 = simplify(inv(R12));
R32 = simplify(inv(R23));
w01 = [0;0;dx1];
w12 = [0;0;dx2];
w23 = [0;0;dx3];

%Angular Velocity Propagation for revolute joints
w11 = R10*w01;
w22 = R21*(w11+w12);
w33 = R32*(w22+w23);

%Linear Velocity propagation for Revolute Joints

r01 = [0;0;l1];
r12 = [l2*cos(x2);l2*sin(x2);0];
r23 = l3*[cos(x3);sin(x3);0];

Rv1 = R10*0;
Rv2 = R21*v01;
Rv3 = R32*v02;

v01 = simplify(Rv1(:,1) + cross(w11, R10*r01,1));
v02 = simplify(Rv2(:,1) + cross(w22, R21*r12, 1));
v03 = simplify(Rv3(:,1) + cross(w33, R32*r23,1));

J = [v01 v02 v03; w11 w22 w33];
