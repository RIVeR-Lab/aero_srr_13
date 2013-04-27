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



T07 = J1*J2*J3*J3_Offset*J4*J5*J6*J7;





T01 = J1;
T02 = T01 * J2;
T03 = T02 * J3;
T04 = T03 * J4*J3_Offset;
T05 = T04 * J5;
T06 = T05 * J6;

FX = T07(1,4);
FY = T07(2,4);
FZ = T07(3,4);


R1 = T01(1:3,3);
R2 = T02(1:3,3);
R3 = T03(1:3,3);
R4 = T04(1:3,3);
R5 = T05(1:3,3);
R6 = T06(1:3,3);

Jacobian = [diff(FX,A1) diff(FX,A2) diff(FX,A3) diff(FX,A4) diff(FX,A5) diff(FX,A6);
    diff(FY,A1) diff(FY,A2) diff(FY,A3) diff(FY,A4) diff(FY,A5) diff(FY,A6);
    diff(FZ,A1) diff(FZ,A2) diff(FZ,A3) diff(FZ,A4) diff(FZ,A5) diff(FZ,A6);
    R1          R2          R3          R4          R5          R6         ];

Jacobian_inv = inv(Jacobian);

i1=Jacobian_inv(1,1)
i2 = Jacobian_inv(1,2)
i3 = Jacobian_inv(1,3)
i4 = Jacobian_inv(1,4)
i5 = Jacobian_inv(1,5)
i6 = Jacobian_inv(1,6)



%num_jacob = subs(Jacobian, A1, 30*pi/180);
%num_jacob=subs(num_jacob, A2, 15*pi/180);
%num_jacob=subs(num_jacob, A3, 10*pi/180);
%num_jacob=subs(num_jacob, A4, 4*pi/180);
%num_jacob=subs(num_jacob, A5, 20*pi/180);
%num_jacob=subs(num_jacob, A6, 30*pi/180);

%num_jacob
%inv(jacob)

end

