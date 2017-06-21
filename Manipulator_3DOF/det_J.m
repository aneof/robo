syms q1 q2 q3
syms l0 l1 l2 l3 l4 l5 real;

J = [ -l2*sin(q1)+(l3+l4*cos(q2)+l5*cos(q2+q3))*cos(q1) , -(l4*sin(q2)+l5*sin(q2+q3))*sin(q1) , -l5*sin(q2+q3)*sin(q1) ;
        0                                                 , l4*cos(q2)+l5*cos(q2+q3)            , l5*cos(q2+q3) ;
        l2*cos(q1)+(l3+l4*cos(q2)+l5*cos(q2+q3))*sin(q1) , (l4*sin(q2)+l5*sin(q2+q3))*cos(q1)  , l5*sin(q2+q3)*cos(q1) ];
detJ = det(J)
simplify(detJ , 'Steps' , 1000)