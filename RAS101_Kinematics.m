%RAS 101 Forward and Inverse Kinematics

syms the1 the2 the3

a1 = 0, a2 = 147, a3 = 135 % sample values

 Px = cos(the1)*(a3*cos(the2 + the3) + a2*cos(the2))

 Py = sin(the1)*(a3*cos(the2 + the3) + a2*cos(the2))

 Pz = a1 + a3*sin(the2 + the3) + a2*sin(the2)

t1=0/180*pi;

t2=35/180*pi;

t3=-60/180*pi;

 px_new=vpa(subs(Px,[the1,the2,the3],[t1,t2,t3]))

 py_new=vpa(subs(Py,[the1,the2,the3],[t1,t2,t3]))
 
 pz_new=vpa(subs(Pz,[the1,the2,the3],[t1,t2,t3]))
 
%Computing IK Solution

[t1,t2,t3] = solve(Px==px_new,Py==py_new,Pz==pz_new,the1,the2,the3)

Theta1=vpa((t1)*180/pi)

Theta2=vpa((t2)*180/pi)

Theta3=vpa((t3)*180/pi)

Solution=[Theta1 Theta2 Theta3]
%%
%%
%SCARA robot

clear all clc

syms  t1 t2 d3 the1 the2 dd3

a0=50; a1=100; a2=150; a3=175;

H01=[ cos(t1) -sin(t1)  0    a1*cos(t1);...
      sin(t1)  cos(t1)  0    a1*sin(t1);...
      0          0      1    a0;...
      0          0      0    1 ]

H12=[ cos(t2) -sin(t2)  0    a3*cos(t2);...
      sin(t2)  cos(t2)  0    a3*sin(t2);...
      0          0      1    a2;...
      0          0      0    1 ]

H23=[ 1   0   0    0;...
      0   1   0    0;...
      0   0   1    -d3;...
      0   0   0    1 ]

H03=H01*H12*H23

Px= H03(1,4)
Py= H03(2,4)
Pz= H03(3,4)

tt1=0/180*pi; %Theta 1

tt2=35/180*pi; % Theta2

d33 =50% Theta 3
 
 %

 px_new=vpa(subs(Px,[t1,t2,d3],[tt1,tt2,d33]))

 py_new=vpa(subs(Py,[t1,t2,d3],[tt1,tt2,d33]))
 
 pz_new=vpa(subs(Pz,[t1,t2,d3],[tt1,tt2,d33]))


%IK correct Solution

 [the1,the2] = solve(Px==px_new,Py==py_new,t1,t2)

Theta1=vpa((the1)*180/pi)

Theta2=vpa((the2)*180/pi)

Solution=[Theta1 Theta2]
 