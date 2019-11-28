%% ROBOTICS STRUCTURE INITIALIZATION
%  11.2019, @Giuseppe Sensolini

clear all
clc

%% Symbolic parameters
syms alpha beta gamma
syms nx ny nz
syms sx sy sz

%% Euler angles test
% ROLL-PITCH-YAW
% RPY(a,b,c) = Rzx'y" =  Rz * Ry * Rx = Rzxy = (Rx.' * Ry.' * Rz.').'
RPY(alpha, beta, gamma);
Rzyz(alpha, beta, gamma);

%% Axis-Angle test

% direct problem (given an angle and axis compute R) TEST ME!!!!
theta = -pi/6
r = [1/sqrt(3); -1/sqrt(3); 1/sqrt(3)]
R = axisangle2rotm(r, theta)

% inverse problem (given R compute an angle and axis)
[r2, theta2] = rotm2axisangle(R)


%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%%===   Functions   =======================================================
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

%% Elementary Rotation matrices
function Rx = Rx(a)
    Rx = [1,0,0; 0,cos(a),-sin(a); 0,sin(a),cos(a)];
end

function Ry = Ry(a)
    Ry = [cos(a),0,sin(a); 0,1,0; -sin(a),0,cos(a)];
end

function Rz = Rz(a)
    Rz = [cos(a),-sin(a),0; sin(a),cos(a),0; 0,0,1];
end

%% Roll-pitch-yaw
% RPY(a,b,c) = Rzx'y" =  Rz * Ry * Rx = Rzxy = (Rx.' * Ry.' * Rz.').'
function RPY = RPY(a, b, c)
    RPY = Rz(a) * Ry(b) * Rx(c);
end

function Rzyz = Rzyz(a, b, c)
    Rzyz = Rz(a) * Ry(b) * Rz(c); 
end

%% given a 3x1 array return a Skew-symmetric matrix
function S = S(r)
    if ~( (size(r,1)==1 && size(r,2)==3)|| (size(r,1)==3 && size(r,2)==1) )
        disp('Skew symmetric function accept only 1x3 or 3x1 array!');
    elseif (size(r,1)==1 && size(r,2)==3)
        r = r';
    end
    S = [   0      -r(3)   r(2) ;
            r(3)   0       -r(1);
            -r(2)  r(1)    0    ];    
end

%% AXIS-ANGLE ---> ROTATION MATRIX
% input 1: axis of rotation
% input 2: angle of rotation (in rad)
% output: a rotation matrix
function R = axisangle2rotm(r, theta)
    R = (r*r') + (eye(3)-r*r')*cos(theta) + S(r)*sin(theta);
end


%%  ROTATION MATRIX ---> AXIS-ANGLE
% input 1: Rotation matrix
% output 1: axis
% output 2: angle
function [r,theta] = rotm2axisangle(R)
    cos_theta = (R(1,1)+R(2,2)+R(3,3)-1)/2;
    sin_theta = 1/2 * sqrt( (R(1,2)-R(2,1))^2 + (R(1,3)-R(3,1))^2 + (R(2,3)-R(3,2))^2 );
    %theta = atan2(sin_theta, cos_theta);
    theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
    r = (1/(2*sin(theta))) * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
end

%{
% check if a given matrix is a right-handed rotation matrix
function is_rotm = is_rotm(R)
    %is a rotation matrix if orthogonal and it's determinant is 1
    is_rotm = false
    id = eye(3)
    if (R*R.'==id && R.'*R==id) && det(R) == 1
        is_rotm = true
    end   
end
%}

%% Compute linear part of the geometric jacobian
% input 1: Transformation matrix (4x4)
% input 2: joints symbolic variables (column vector)
% output: 3xN matrix
function Jl = Jl(T, q)
   Jl = [];
   n = size(q); n = n(1);   %number of joints
   for i = 1:n
       Jl = [Jl, simplify( diff(T(1:3,4), q(i)) )];   
   end
end