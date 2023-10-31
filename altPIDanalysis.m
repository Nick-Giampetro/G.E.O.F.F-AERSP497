clc ;
clear ;

m = 1 ;
g = 9.81 ;

A = [ 0 1 ;
      0 0 ] ;

B = [ 0   ;
      1/m ] ;

C = [1 0] ;

D = 0 ;

[num,den] = ss2tf(A,B,C,D);

G = tf(num,den) ;

rltool(G) ; % add two zeros and one pole at the origin to make a PID

% THIS IS SUPER SIMPLIFIED JUST SOMETHING TO HELP ME (Nick) think
