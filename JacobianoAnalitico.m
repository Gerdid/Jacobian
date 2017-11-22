clc;clear

% Datos y variables
syms q1 q2 q3 q4 q5 q6 q7 J x y z phi theta psi
d1 = 0;
d2 = 0;
d3 = 3;
d4 = 0;
d5 = 5;
d6 = 0;
d7 = 7;
alfa1 = deg2rad(-90);
alfa2 = deg2rad(90);
alfa3 = deg2rad(-90);
alfa4 = deg2rad(90);
alfa5 = deg2rad(-90);
alfa6 = deg2rad(90);
alfa7 = deg2rad(0);
a1 = 0;
a2 = 0;
a3 = 0;
a4 = 0;
a5 = 0;
a6 = 0;
a7 = 0;

% Matrices de DH
A01=[cos(q1),-cos(alfa1)*sin(q1),sin(alfa1)*sin(q1),a1*cos(q1);sin(q1),cos(alfa1)*cos(q1),-sin(alfa1)*cos(q1),a1*sin(q1);0,sin(alfa1),cos(alfa1),d1;0,0,0,1];
A12=[cos(q2),-cos(alfa2)*sin(q2),sin(alfa2)*sin(q2),a2*cos(q2);sin(q2),cos(alfa2)*cos(q2),-sin(alfa2)*cos(q2),a2*sin(q2);0,sin(alfa2),cos(alfa2),d2;0,0,0,1];
A23=[cos(q3),-cos(alfa3)*sin(q3),sin(alfa3)*sin(q3),a3*cos(q3);sin(q3),cos(alfa3)*cos(q3),-sin(alfa3)*cos(q3),a3*sin(q3);0,sin(alfa3),cos(alfa3),d3;0,0,0,1];
A34=[cos(q4),-cos(alfa4)*sin(q4),sin(alfa4)*sin(q4),a4*cos(q4);sin(q4),cos(alfa4)*cos(q4),-sin(alfa4)*cos(q4),a4*sin(q4);0,sin(alfa4),cos(alfa4),d4;0,0,0,1];
A45=[cos(q5),-cos(alfa5)*sin(q5),sin(alfa5)*sin(q5),a5*cos(q5);sin(q5),cos(alfa5)*cos(q5),-sin(alfa5)*cos(q5),a5*sin(q5);0,sin(alfa5),cos(alfa5),d5;0,0,0,1];
A56=[cos(q6),-cos(alfa6)*sin(q6),sin(alfa6)*sin(q6),a6*cos(q6);sin(q6),cos(alfa6)*cos(q6),-sin(alfa6)*cos(q6),a6*sin(q6);0,sin(alfa6),cos(alfa6),d6;0,0,0,1];
A67=[cos(q7),-cos(alfa7)*sin(q7),sin(alfa7)*sin(q7),a7*cos(q7);sin(q7),cos(alfa7)*cos(q7),-sin(alfa7)*cos(q7),a7*sin(q7);0,sin(alfa7),cos(alfa7),d7;0,0,0,1];

A07=A01*A12*A23*A34*A45*A56*A67;
MH(q1,q2,q3,q4,q5,q6,q7)=A01*A12*A23*A34*A45*A56*A67;

% Ecuaciones para cinemática directa
x = A07(1,4);
y = A07(2,4);
z = A07(3,4);
phi = atan2(A07(2,1),A07(1,1));
theta = atan2(-A07(3,1),sqrt((A07(1,1))^2+(A07(2,1))^2));
psi= atan2(A07(3,2),A07(3,3));

% Jacobiana
Ja(q1,q2,q3,q4,q5,q6,q7) = [diff(x,q1) diff(x,q2) diff(x,q3) diff(x,q4) diff(x,q5) diff(x,q6) diff(x,q7); diff(y,q1) diff(y,q2) diff(y,q3) diff(y,q4) diff(y,q5) diff(y,q6) diff(y,q7); diff(z,q1) diff(z,q2) diff(z,q3) diff(z,q4) diff(z,q5) diff(z,q6) diff(z,q7); diff(phi,q1) diff(phi,q2) diff(phi,q3) diff(phi,q4) diff(phi,q5) diff(phi,q6) diff(phi,q7); diff(theta,q1) diff(theta,q2) diff(theta,q3) diff(theta,q4) diff(theta,q5) diff(theta,q6) diff(theta,q7); diff(psi,q1) diff(psi,q2) diff(psi,q3) diff(psi,q4) diff(psi,q5) diff(psi,q6) diff(psi,q7)];

% Valores de q
q1 = deg2rad(70);
q2 = deg2rad(20);
q3 = deg2rad(-30);
q4 = deg2rad(60);
q5 = deg2rad(-15);
q6 = deg2rad(75);
q7 = deg2rad(10);


% Resultado Matriz homogénea
vpa(MH(q1,q2,q3,q4,q5,q6,q7),3)

% Resultado [x,y,z,phi,theta,psi]
vpa(Ja(q1,q2,q3,q4,q5,q6,q7)*[q1;q2;q3;q4;q5;q6;q7],4)