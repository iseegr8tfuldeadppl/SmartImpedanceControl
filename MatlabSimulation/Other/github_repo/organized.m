syms L1 L2 m1 m2 d1 d2 I1 I2 g th1 th2 thdotdot1 thdotdot2 thdot1 thdot2 real
syms Fdx Fdy real

Fd = [Fdx, Fdy];

P1 = [ - L2*cos(th1 + th2) - L1*cos(th1) ;
       - L2*sin(th1 + th2) - L1*sin(th1) ];
   
P2 = [ - L2*cos(th1 + th2) ;
       - L2*sin(th1 + th2) ];
 

M = [ I2 + I1 + 2*m2*d2*L1*cos(th2) + m2*L1^2 + m2*d2^2 + m1*d1^2  , I2 + m2*d2^2  + m2*d2*L1*cos(th2) ;
      I2 + d2*m2*L1*cos(th2) + m2*d2^2                             , I2 + m2*d2^2                      ];
          
B = [ - d2*m2*L1*thdot2^2*sin(th2) - 2*m2*d2*L1*thdot1*thdot2*sin(th2) ;
        d2*m2*L1*sin(th2)*thdot1^2                                     ];
              
G = [ - g*m2*d2*sin(th1 + th2) - (m2*L1 + d1*m1)*g*sin(th1) ;
      - d2*g*m2*sin(th1 + th2)                              ];
              
F = [ Fd*P1 ;
      Fd*P2 ];

T = [ T1 ;
      T2 ];
  
  
Zero = M * [thdotdot1;thdotdot2] + B + G - [T1;T2] - F;

%Zero =  M^-1 * ( [thdotdot1;thdotdot2] + B + G - [T1;T2] - F );

eqn1forthdotdot1 = solve(Zero(1), thdotdot1);
eqn2forthdotdot2 = solve(Zero(2), thdotdot2);

eqn1 = simplify(solve(subs(eqn1forthdotdot1, thdotdot2, eqn2forthdotdot2)-thdotdot1,thdotdot1));
eqn2 = simplify(solve(subs(eqn2forthdotdot2, thdotdot1, eqn1forthdotdot1)-thdotdot2,thdotdot2));

