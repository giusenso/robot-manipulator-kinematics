%% RRPRP robot KINEMATICS
%  07.2019, @Giuseppe Sensolini
%  my solution to the Robotics 1 exam[March 27, 2018, De Luca, La Sapienza]

clear all
clc

%% symbolic parameters
syms a alpha d theta    
N = 5                    
q = sym('q',[N,1])
d1 = sym('d1');

   %% Build the general Denavit-Hartenberg trasformation matrix
DH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];
%% Denavit-Hartemberg table
DHTABLE = [     pi/2    0   d1      q(1);
                pi/2    0   0       q(2);
                pi/2    0   q(3)    pi;
                pi/2    0   0       q(4);
                0       0   q(5)    0     ]

%% Build transformation matrices for each link
A = cell(1,N);
for i = 1:N
    alpha   = DHTABLE(i,1);
    a       = DHTABLE(i,2);
    d       = DHTABLE(i,3);
    theta   = DHTABLE(i,4);
    A{i} = subs(DH);
end

%% Build base-to-end-effector transformation matrix
A05 = eye(4);
for i = 1:N 
    A05 = A05 * A{i};
    A05 = simplify(A05);
end

%% output

P05 = A05(1:3, 4)   % poition vector

R05 = A05(1:3, 1:3) % rotation matrix

A05 % base-to-end-effector transformation matrix


%% compute Jacobian matrix

revolute_joint = [1 1 0 1 0]
prismatic_joint = ~revolute_joint

% linear component
Jl = zeros(0,0);
for i = 1:N
   Jl = simplify( [Jl , diff(P05,q(i))] );
end

% angular component
Ja = zeros(0,0);
R = eye(3);
z = [0 0 1]';
for i = 1:N
    Ja =  ( [Ja , R * z * revolute_joint(i)] );
    R = simplify( R * (A{i}(1:3,1:3)) );
end

% full Jacobian
J = [Jl ; Ja]

J_det = simplify(det(Jl(1:N,1:N)))

J_null = simplify(null(J))
