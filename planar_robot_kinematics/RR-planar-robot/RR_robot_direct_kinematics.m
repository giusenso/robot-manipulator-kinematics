%% 2R planar robot DIRECT KINEMATICS
%  07.2018, @Giuseppe Sensolini

clear all
clc

%% define symbolic variables
syms a alpha d theta

%% number of joint
N = 2;

%% DH table of parameters of 2R, same for each joint
% assign these parameters in order to compute specific values
%          | a | alpha | d | theta |
DHTABLE = [ sym('a1')   0   0   sym('q1');
            sym('a2')   0   0   sym('q2')];
        
%% Build the general Denavit-Hartenberg trasformation matrix
TDH = [ cos(theta) -sin(theta)      0	a*cos(theta);
        sin(theta)  cos(theta)      0   a*sin(theta);
          0             0           1       d		;
          0             0           0       1       ];

%% Build transformation matrices for each link
A = cell(1,N);

% For every row in 'DHTABLE' we substitute the right value inside
% the general DH matrix
for i = 1:N
    a = DHTABLE(i,1);
    alpha = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
end

%% Direct kinematics
disp('Direct kinematics of 2R robot in symbolic form')
disp(['Number of joints N = ',num2str(N)])
T = eye(4);

for i = 1:N 
    T = T * A{i};
    T = simplify(T);
end

% output direct kinematics matrix
A05 = T

% output end-effector position
P05 = T(1:3,4)

%% compute Jacobian matrix

% linear component
Jl = simplify( [diff(P05,DHTABLE(1,4)) , diff(P05,DHTABLE(2,4))] );

% angular component
Ja = zeros(0,0);
R = eye(3);
z = [0 0 1]';
Ja =  simplify( [ A{1}(1:3,1:3)*z , A{1}(1:3,1:3)*A{2}(1:3,1:3)*z] );

% full Jacobian
J = [Jl ; Ja]

J_det = simplify(det(Jl(1:N,1:N)))

%% compute joint velocities
Ve = sym('Ve',[N,1])  %end effector velocity

joint_velocity = simplify( inv(J(1:N,1:N)) * Ve )



%% end
