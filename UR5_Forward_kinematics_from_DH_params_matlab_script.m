syms t1 t2 t3 t4 t5 t6 
Q=[0            ,0          ,89         ,t1-pi;
   -pi/2        ,0          ,0          ,t2;
   0            ,425        ,109.75     ,t3;
   0            ,392        ,0          ,t4+pi;
   pi/2         ,0          ,94.75      ,t5;
   -pi/2        ,0          ,81.5       ,t6];
N=size(Q,1);
B=1;
for i=1:N
A= [cos(Q(i,4))            ,-sin(Q(i,4))           ,0           ,Q(i,2)             ;
    sin(Q(i,4))*cos(Q(i,1)),cos(Q(i,4))*cos(Q(i,1)),-sin(Q(i,1)),-sin(Q(i,1))*Q(i,3);
    sin(Q(i,4))*sin(Q(i,1)),cos(Q(i,4))*sin(Q(i,1)),cos(Q(i,1)) ,cos(Q(i,1))*Q(i,3) ;
    0                      ,0                      ,0           ,1                 ];
B=B*A;
B=simplify(B);
end
B=vpa(B)
