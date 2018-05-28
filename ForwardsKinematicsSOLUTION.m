function [outputArray] = ForwardsKinematicsSOLUTION(t1,t2,t3,t4,t5,t6)

% Replace the '0's in the previous lines with joint values in radians

DHparams=[0            ,0          ,89.2       ,t1-pi;
          -pi/2        ,0          ,0          ,t2;
          0            ,425        ,0     ,t3;
          0            ,392        ,109.75          ,t4+pi;
          pi/2         ,0          ,94.75      ,t5;
          -pi/2        ,0          ,81.5       ,t6];
      
X = 109.75*sin(t1) - 425.0*cos(t1)*cos(t2) + 81.5*cos(t5)*sin(t1) + 94.75*sin(t2 + t3 + t4)*cos(t1) - 392.0*cos(t2 + t3)*cos(t1) - 81.5*cos(t2 + t3 + t4)*cos(t1)*sin(t5);
Y = 94.75*sin(t2 + t3 + t4)*sin(t1) - 81.5*cos(t1)*cos(t5) - 425.0*cos(t2)*sin(t1) - 109.75*cos(t1) - 392.0*cos(t2 + t3)*sin(t1) - 81.5*cos(t2 + t3 + t4)*sin(t1)*sin(t5);
Z = 89.0 - 392.0*sin(t2 + t3) - 425.0*sin(t2) - 181.5*sin(t2 + t3 + t4)*sin(t5) - 94.75*cos(t2 + t3 + t4);


% x bits
r11 = cos(t6)*(sin(t1)*sin(t5) + cos(t2 + t3 + t4)*cos(t1)*cos(t5)) - 1.0*sin(t2 + t3 + t4)*cos(t1)*sin(t6);
r12 = - 1.0*sin(t6)*(sin(t1)*sin(t5) + cos(t2 + t3 + t4)*cos(t1)*cos(t5)) - 1.0*sin(t2 + t3 + t4)*cos(t1)*cos(t6);
r13 = cos(t5)*sin(t1) - 1.0*cos(t2 + t3 + t4)*cos(t1)*sin(t5);

% y bits

r21 =  - 1.0*cos(t6)*(cos(t1)*sin(t5) - 1.0*cos(t2 + t3 + t4)*cos(t5)*sin(t1)) - 1.0*sin(t2 + t3 + t4)*sin(t1)*sin(t6);
r22 = sin(t6)*(cos(t1)*sin(t5) - 1.0*cos(t2 + t3 + t4)*cos(t5)*sin(t1)) - 1.0*sin(t2 + t3 + t4)*cos(t6)*sin(t1);
r23 = - 1.0*cos(t1)*cos(t5) - 1.0*cos(t2 + t3 + t4)*sin(t1)*sin(t5);

% z bits
r31 = cos(t2 + t3 + t4)*sin(t6) + sin(t2 + t3 + t4)*cos(t5)*cos(t6);
r32 = cos(t2 + t3 + t4)*cos(t6) - 1.0*sin(t2 + t3 + t4)*cos(t5)*sin(t6);
r33 = -1.0*sin(t2 + t3 + t4)*sin(t5);

%   Euler angles;
rotX = atan2(r32,r33);
rotY = atan2(-r31, sqrt(r32^2 + r33^2));
rotZ = atan2(r21, r11);
disp("From initial joint values in radians: ");
disp(t1+" | "+t2+" | "+t3+" | "+t4+" | "+t5+" | "+t6);
disp("X: "+X+" | Y: "+Y+" | Z: "+Z);
disp("rotX: "+rotX+" | rotY: "+rotY+" | rotZ: "+rotZ);
outputArray = [X,Y,Z,rotX,rotY,rotZ];
end
