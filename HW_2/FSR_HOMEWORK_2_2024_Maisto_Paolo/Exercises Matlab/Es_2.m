clear all;
close all;
clc;

%% Data
r=0.4;
l=0.6;

%% Coordinates and inputs
syms x;
syms y;
syms theta;
syms alfa;
syms beta;
syms gamma;
q=[x;y;theta;alfa;beta;gamma];
[n,~]=size(q);

syms u1;
syms u2;
syms u3;

%% Matrix A^T
A=[(sqrt(3)*cos(theta))/2-sin(theta)/2  cos(theta)/2+(sqrt(3)*sin(theta))/2  l r 0 0;
    sin(theta)                              -cos(theta)                      l 0 r 0;
    (-sqrt(3)*cos(theta))/2-sin(theta)/2 cos(theta)/2-(sqrt(3)*sin(theta))/2 l 0 0 r];

%% Matrix G
G=null(A);
[~,m]=size(G);
g1=G(:,1);
g2=G(:,2);
g3=G(:,3);

%% Kinematic Model
disp("________________")
disp("Kinematic Model")
qdot=G*[u1;u2;u3]
disp("________________")

%% Holonomy check

% Computing the Jacobian refering the 6 coordinates
dg1=jacobian(g1,q);
dg2=jacobian(g2,q);
dg3=jacobian(g3,q);

% Computing the lie bracket
Lg12=dg2*g1-dg1*g2;
Lg13=dg3*g1-dg1*g3;
Lg23=dg3*g2-dg2*g3;

% Accessibility Distribution
delta_a=[g1 g2 g3 Lg12 Lg13 Lg23];
dim_a=size(delta_a);

% Computing the rank to verify they are linearly independent
R=rank(delta_a);
if R==n 
    disp("System is completely nonholonomic")
elseif (R>m)&&(R<n)
    disp("System is nonholonomic but partially integrable")
elseif R==m
    disp("System is completely holonomic")
end
