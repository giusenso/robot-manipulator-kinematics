%% PRPR robot KINEMATICS
%  06.2017, @Giuseppe Sensolini
%  my solution to the Robotics 1 exam[February 5, 2018, De Luca, La Sapienza]

clear all
clc

%% Parameters
N = 3;   %joint number
revolute = [1 1 1];
prismatic = ~revolute;
q = sym('q', [N,1]);
syms d1 a2 a3 ;

%% DH table
DHTABLE = [ pi/2	0       d1      q(1);
            0       a2      0       q(2);
            0       a3      0       q(3)]


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

A03 = eye(4);
for i = 1:N
    A{i};
    A03  = A03 * A{i};
end
simplify(A03)

%% compute geometric Jacobian
Jl = [];
Ja = [];
R = eye(3);
for i = 1:N
	Jl = [Jl, simplify( diff(A03(1:3,4),q(i)) )];
    Ja = [Ja , R * [0 0 1]' * revolute(i)];
    R = simplify( R * (A{i}(1:3,1:3)) );
end
J = simplify([Jl ; Ja])