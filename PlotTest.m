close all
A = [];
B = [];
for i = loghistory1.'
    output = ForwardsKinematicsSOLUTION(i(1),i(2),i(3),i(4),i(5),i(6));
    A=[A 
       output];
end
for i = Curve.'
    output = ForwardsKinematicsSOLUTION(i(1),i(2),i(3),i(4),i(5),i(6));
    B=[B 
       output];
end
x1 = A(:,1);
y1 = A(:,2);
z1 = A(:,3);
line(x1,y1)
hold on
x2 = B(:,1);
y2 = B(:,2);
z2 = B(:,3);
line(x2,y2)