%%  Robot: GMF M-100, a 5-dof manipulator with a RPP-RR joints
%  08.2018, @Giuseppe Sensolini
%  my solution to the Robotics 1 exam[February 5, 2018, De Luca, La Sapienza]

clear all
clc

%% Parameters
N = 5;   %joint number
revolute = [1 0 0 1 1];
prismatic = ~revolute;
q = sym('q', [N,1]);
syms d4;

%% DH table
DHTABLE = [ 0, 0,  0, q(1);
            -pi/2, 0, q(2), -pi/2;
            pi/2, 0, q(3), 0;
            -pi/2, 0, d4, q(4);
            0, 0,  0,    q(5)]


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

T = eye(4);
for i = 1:N
    A{i}
    T  = T * A{i};
end
% position vector (3x1)
P05 = simplify(T(1:3,4))

% rotation matrix (3x3)
R05 = simplify(T(1:3,1:3))

% transformation matrix (4x4)
simplify(T)



%% compute geometric Jacobian
Jl = [];
Ja = [];
R = eye(3);
for i = 1:N
	Jl = [Jl, simplify( diff(T(1:3,4),q(i)) )];
    Ja = [Ja , R * [0 0 1]' * revolute(i)];
    R = simplify( R * (A{i}(1:3,1:3)) );
end
J = simplify([Jl ; Ja])





