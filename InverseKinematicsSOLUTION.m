%X=-65.947;
%Y=-460.226;
%Z=180.897;
%rotX=-3.108;
%rotY=-0.285;
%rotZ=-0.221;

X=-45.095;
Y=-456.347;
Z=32.408;
rotX=3.141;
rotY=deg2rad(-0.128);
rotZ=deg2rad(10.471);

toolLength = 150;
syms t1 t2 t3 t4 t5 t6
DHparams=[0            ,0          ,89.2       ,t1-pi;
          -pi/2        ,0          ,0          ,t2;
          0            ,425        ,0     ,t3;
          0            ,392        ,109          ,t4+pi;
          pi/2         ,0          ,94.75      ,t5;
          -pi/2        ,0          ,82+toolLength       ,t6];

B = [X
    Y
    Z];
C = [rotZ,rotY,rotX];
Cmatrix = eul2rotm(C);
Null = [0,0,0];

T06 = [
     Cmatrix,B
     Null,1
     ];
% T is 0T6

minusD = [0
    0
    -DHparams(6,3)
    1];
P05 = T06*minusD;
theta1value1 = atan2(P05(2),P05(1))+acos(DHparams(4,3)/(sqrt(P05(1)^2+P05(2)^2)))+pi/2;
theta1value2 = atan2(P05(2),P05(1))-acos(DHparams(4,3)/(sqrt(P05(1)^2+P05(2)^2)))+pi/2;

fprintf("---------------------------\nTheta 1 value 1: "+double(radtodeg(theta1value1))+"\n");
fprintf("Theta 1 value 2: "+double(radtodeg(theta1value2))+"\n---------------------------\n");

P06 = B;

RotTh1Val1 = [double(theta1value1),0,0];
RotTh1Val2 = [double(theta1value2),0,0];

P16va1 = transpose(eul2rotm(RotTh1Val1))*P06;
P16va2 = transpose(eul2rotm(RotTh1Val2))*P06;
%theta5Val1 = acos((P16va1(2)-DHparams(4,3))/DHparams(6,3));
%theta5Val2 = -acos((P16va1(2)-DHparams(4,3))/DHparams(6,3));

%theta5Val3 = acos((P16va2(2)-DHparams(4,3))/DHparams(6,3));
%theta5Val4 = -acos((P16va2(2)-DHparams(4,3))/DHparams(6,3));

preVal1 = (X*sin(theta1value1)-Y*cos(theta1value1)-DHparams(4,3))/DHparams(6,3);

theta5value11 = acos(preVal1);
theta5value12 = -acos(preVal1);

if abs(preVal1)>1
   disp("Joint 5 has non-real value 1 position. Not sure what this means in practise");
end

preVal2 = (X*sin(theta1value2)-Y*cos(theta1value2)-DHparams(4,3))/DHparams(6,3);

theta5value21 = acos(preVal2);
theta5value22 = -acos(preVal2);

if abs(preVal2)>1
   disp("Joint 5 has non-real value 2 position. Not sure what this means in practise");
end

disp("Theta 5 value 11: "+double(radtodeg(theta5value11)));
disp("Theta 5 value 12: "+double(radtodeg(theta5value12)));

disp("Theta 5 value 21: "+double(radtodeg(theta5value21)));
disp("Theta 5 value 22: "+double(radtodeg(theta5value22)));


% Finding angle 6:

T60 = inv(T06);
%note to self: ask rasmus about how to properly find P60 without 
%inversing the entire transormation

val6Numerator11 = -T60(1,2)*sin(theta1value1)+T60(2,2)*cos(theta1value1);
val6Numerator12 = -T60(1,2)*sin(theta1value2)+T60(2,2)*cos(theta1value2);

val6Numerator21 = T60(1,1)*sin(theta1value1)-T60(2,1)*cos(theta1value1);
val6Numerator22 = T60(1,1)*sin(theta1value2)-T60(2,1)*cos(theta1value2);

val6Denominator11 = sin(theta5value11);
val6Denominator12 = sin(theta5value12);

val6Denominator21 = sin(theta5value21);
val6Denominator22 = sin(theta5value22);


theta6value11 = atan2(val6Numerator11/val6Denominator11,val6Numerator21/val6Denominator11);
theta6value12 = atan2(val6Numerator11/val6Denominator12,val6Numerator21/val6Denominator12);

theta6value21 = atan2(val6Numerator12/val6Denominator21,val6Numerator22/val6Denominator21);
theta6value22 = atan2(val6Numerator12/val6Denominator22,val6Numerator22/val6Denominator22);
%disp(double(rad2deg(theta6value1)));
fprintf("---------------------------\nTheta 6 value 11: "+rad2deg(double(theta6value11))+"\n");
fprintf("Theta 6 value 12: "+rad2deg(double(theta6value12))+"\n");
fprintf("Theta 6 value 21: "+rad2deg(double(theta6value21))+"\n");
fprintf("Theta 6 value 22: "+rad2deg(double(theta6value22))+"\n---------------------------\n");
if val6Denominator11==0
    disp("Joint 6 has a non real value 1 1 position")
end

if val6Denominator12==0
    disp("Joint 6 has a non real value 1 2 position")
end   

if val6Denominator21==0
    disp("Joint 6 has a non real value 2 1 position")
end

if val6Denominator22==0
    disp("Joint 6 has a non real value 2 2 position")
end

%Finding joint 3:

% I don't understand this very well. However, this is what im currently
% thinking:
% 
% T06 = T01 * T14 * T45 * T56
% I failed in that approach because idk how to remove the rotations from
% the end
% 
% we can get P16 by appling the transposed rotation to P06
% we can get P05 by doing ?Whatever? we did for theta 1
% maybe we can get P04 by doing that again with d5? but we have to make a
% transformation instead of just the P bit 

%R56_11 = eul2rotm([theta5value11,0,0]);
%R56_12 = eul2rotm([theta5value12,0,0]);
%R56_21 = eul2rotm([theta5value21,0,0]);
%R56_22 = eul2rotm([theta5value22,0,0]);

%T05 = [ somerotation, P05
%        Null,1 ];
% in the above line we need to find the value of 'somerotation'    
    
P56 = [0
    DHparams(6,3)
    0];

RotTh6Val11 = eul2rotm([double(theta6value11),0,0]);
RotTh6Val12 = eul2rotm([double(theta6value12),0,0]);

RotTh6Val21 = eul2rotm([double(theta6value21),0,0]);
RotTh6Val22 = eul2rotm([double(theta6value22),0,0]);

T56_11 = [RotTh6Val11,P56
          Null,1];

T56_12 = [RotTh6Val12,P56
          Null,1];

T56_21 = [RotTh6Val21,P56
          Null,1];

T56_22 = [RotTh6Val22,P56
          Null,1];


P45 = [0
  -DHparams(5,3)  
0];

RotTh5Val11 = eul2rotm([double(theta5value11),0,0]);
RotTh5Val12 = eul2rotm([double(theta5value12),0,0]);

RotTh5Val21 = eul2rotm([double(theta5value21),0,0]);
RotTh5Val22 = eul2rotm([double(theta5value22),0,0]);

T45_11 = [RotTh5Val11,P45
          Null,1];

T45_12 = [RotTh5Val12,P45
          Null,1];

T45_21 = [RotTh5Val21,P45
          Null,1];

T45_22 = [RotTh5Val22,P45
          Null,1];

      
P01 = [ DHparams(1,3)
        0
        0 ];
T01_1 = [ eul2rotm(RotTh1Val1),P01
          Null,1 ];
T01_2 = [ eul2rotm(RotTh1Val2),P01
          Null,1 ];
      

T14_11 = inv(T01_1)*T06*inv(T56_11)*inv(T45_11);
T14_12 = inv(T01_1)*T06*inv(T56_12)*inv(T45_12);
T14_21 = inv(T01_2)*T06*inv(T56_21)*inv(T45_21);
T14_22 = inv(T01_2)*T06*inv(T56_22)*inv(T45_22);

theta3value111 = acos((abs(sqrt(T14_11(1,4)^2+T14_11(2,4)))^2-DHparams(3,2)^2-DHparams(4,2)^2)/(2*DHparams(3,2)*DHparams(4,2)));
theta3value112 = -acos((abs(sqrt(T14_11(1,4)^2+T14_11(2,4)))^2-DHparams(3,2)^2-DHparams(4,2)^2)/(2*DHparams(3,2)*DHparams(4,2)));

theta3value121 = acos((abs(sqrt(T14_12(1,4)^2+T14_11(2,4)))^2-DHparams(3,2)^2-DHparams(4,2)^2)/(2*DHparams(3,2)*DHparams(4,2)));
theta3value122 = -acos((abs(sqrt(T14_12(1,4)^2+T14_11(2,4)))^2-DHparams(3,2)^2-DHparams(4,2)^2)/(2*DHparams(3,2)*DHparams(4,2)));

theta3value211 = acos((abs(sqrt(T14_21(1,4)^2+T14_11(2,4)))^2-DHparams(3,2)^2-DHparams(4,2)^2)/(2*DHparams(3,2)*DHparams(4,2)));
theta3value212 = -acos((abs(sqrt(T14_21(1,4)^2+T14_11(2,4)))^2-DHparams(3,2)^2-DHparams(4,2)^2)/(2*DHparams(3,2)*DHparams(4,2)));

theta3value221 = acos((abs(sqrt(T14_22(1,4)^2+T14_11(2,4)))^2-DHparams(3,2)^2-DHparams(4,2)^2)/(2*DHparams(3,2)*DHparams(4,2)));
theta3value222 = -acos((abs(sqrt(T14_22(1,4)^2+T14_11(2,4)))^2-DHparams(3,2)^2-DHparams(4,2)^2)/(2*DHparams(3,2)*DHparams(4,2)));

fprintf("Theta 3 value 111: "+rad2deg(double(theta3value111))+"\n");
fprintf("Theta 3 value 112: "+rad2deg(double(theta3value112))+"\n");
fprintf("Theta 3 value 121: "+rad2deg(double(theta3value121))+"\n");
fprintf("Theta 3 value 122: "+rad2deg(double(theta3value122))+"\n");
fprintf("Theta 3 value 211: "+rad2deg(double(theta3value211))+"\n");
fprintf("Theta 3 value 212: "+rad2deg(double(theta3value212))+"\n");
fprintf("Theta 3 value 221: "+rad2deg(double(theta3value221))+"\n");
fprintf("Theta 3 value 222: "+rad2deg(double(theta3value222))+"\n---------------------------\n");





