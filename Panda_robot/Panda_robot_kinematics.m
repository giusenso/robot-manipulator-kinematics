%% RRPRP robot KINEMATICS
%  08.2018, @Giuseppe Sensolini
%  my solution to the Robotics 1 exam[Jenuary 11, 2018, De Luca, La Sapienza]

clear all
clc

%% symbolic parameters
syms alpha a d theta;  
N = 7;           
q = sym('q',[N,1]);
syms d1 d2 d3 d4 d5 d6 d7
syms a1 a2 a3 a4 a5 a6 a7

   %% Build the general Denavit-Hartenberg trasformation matrix
DH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                      1];
%% Denavit-Hartemberg table
DHTABLE = [     pi/2    0       d1    q(1);
                -pi/2   0       0       q(2);
                pi/2    a3    d3    q(3);
                -pi/2   a4    0       q(4);
                pi/2    0       d5    q(5);
                pi/2    a6    0       q(6);
                0       0       d7    q(7)    ]   

%% Build transformation matrices for each link
A = cell(1,N);
for i = 1:N
    alpha   = DHTABLE(i,1);
    a       = DHTABLE(i,2);
    d       = DHTABLE(i,3);
    theta   = DHTABLE(i,4);
    A{i} = simplify(subs(DH));
end


for i = 1:N
    A{i}
end