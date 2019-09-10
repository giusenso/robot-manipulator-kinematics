%% PRPR robot KINEMATICS
%  06.2017, @Giuseppe Sensolini
%  my solution to the Robotics 1 exam[February 5, 2018, De Luca, La Sapienza]

clear all
clc

%% Parameters
N = 4;   %joint number
revolute = [0 1 0 1];
prismatic = ~revolute;
q = sym('q', [N,1]);
syms l4 ;

%% DH table
DHTABLE = [ -pi/2	0       q(1)    0;
            pi/2	0       0       q(2);
            pi/2	0       q(3)    0;
            0       l4      0       q(4) ]


%% Build the general Denavit-Hartenberg trasformation matrix
syms alpha a d theta
general_DH = [  cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                0             sin(alpha)             cos(alpha)            d;
                0               0                       0                      1];

%% compute transformation matrices
A   = cell(1,N);
for i = 1:N
    alpha   = DHTABLE(i,1);
    a       = DHTABLE(i,2);
    d       = DHTABLE(i,3);
    theta   = DHTABLE(i,4);
    A{i} = simplify(subs(general_DH));
end

A04 = eye(4);
for i = 1:N
    A{i};
    A04  = A04 * A{i};
end
simplify(A04)