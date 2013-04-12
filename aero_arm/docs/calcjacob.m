function [ output_args ] = get_jacobian( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
 syms A1 A2 A3 A4 A5 A6
 
 J1 = [cos(A1) -sin(A1) 0 0; -sin(A1) -cos(A1) 0 0; 0 0 -1 0.1370; 0 0 0 1];

J2 = [sin(A2) cos(A2) 0 0; 0 0 1 0; cos(A2) -sin(A2) 0 -0.1181;0 0 0 1];

 J3 = [-cos(A3) sin(A3) 0 0.41; sin(A3) cos(A3) 0 0; 0 0 -1 0; 0 0 0 1];
 
 J3_Offset = [1 0 0 0; 0 1 0 0; 0 0 1 0.0113; 0 0 0 1];
 
J4 = [0 0 -1 0.207; sin(A4) cos(A4) 0 0; cos(A4) -sin(A4) 0 0; 0 0 0 1];

J5 = [cos(-0.959931089)*cos(A5) cos(-0.959931089)*-sin(A5) sin(-0.959931089) cos(0.959931089*0.075); sin(A5) cos(A5) 0 0; -sin(-0.959931089)*cos(A5) sin(-0.959931089)*sin(A5) cos(-0.959931089) -sin(0.959931089*0.075); 0 0 0 1];

J6 = [cos(0.959931089)*cos(A6) cos(0.959931089)*-sin(A6) sin(0.959931089) -cos(0.959931089*0.075); sin(A6) cos(A6) 0 0; -sin(0.959931089)*cos(A6) sin(0.959931089)*sin(A6) cos(0.959931089) -sin(0.959931089*0.075); 0 0 0 1];

J7 = [1 0 0 0; 0 1 0 0; 0 0 1 -0.1850; 0 0 0 1];



Trans = J1*J2*J3*J3_Offset*J4*J5*J6*J7;

V = [A1 A2 A3 A4 A5 A6];

jacob = jacobian(Trans);

num_jacob = subs(jacob, A1, 30*pi/180);
num_jacob=subs(num_jacob, A2, 15*pi/180);
num_jacob=subs(num_jacob, A3, 10*pi/180);
num_jacob=subs(num_jacob, A4, 4*pi/180);
num_jacob=subs(num_jacob, A5, 20*pi/180);
num_jacob=subs(num_jacob, A6, 30*pi/180);

num_jacob
%inv(jacob)

end

