

sym x_1;
sym x_2; sym x_3; sym x_4; sym x_5; sym x_6; sym d_1;
sym d_6; sym a_2; sym a_3; sym a_4;

T1 = [cosd(x_1) 0 sind(x_1) 0; sin(x_1) 0 -cos(x_1) 0; 0 1 0 d_1;0 0 0 1];

S1 = simplify(T1);

T2 = [cosd(x_2) -sind(x_2) 0 a_2*cosd(x_2); sind(x_2) cosd(x_2) 0 a_2*sind(x_2); 0 0 1 0;0 0 0 1];

S2 = simplify(T1*T2);

T3 = [cosd(x_3) -sind(x_3) 0 a_3*cosd(x_3); sind(x_3) cosd(x_3) 0 a_3*sind(x_3); 0 0 1 0;0 0 0 1];

S3 = simplify(T1*T2*T3);

T4 = [cosd(x_4) 0 -sind(x_4) a_4*cosd(x_4); sind(x_4) 0 cosd(x_4) a_4*sind(x_4); 0 -1 0 0;0 0 0 1];

S4 = simplify(T1*T2*T3*T4);

T5 = [cosd(x_5) 0 -sind(x_5) 0;sind(x_5) 0 cosd(x_5) 0;0 -1 0 0;0 0 0 1];

S5 = simplify(T1*T2*T3*T4*T5);

T6 = [cosd(x_6) -sind(x_6) 0 0;sind(x_6) cosd(x_6) 0 0;0 0 1 d_6; 0 0 0 1];

To = T1*T2*T3*T4*T5*T6;

S0 = simplify(To);
