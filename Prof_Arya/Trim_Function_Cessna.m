          % u v w phi theta psi x y z p  q  r
          % 1 2 3 4   5     6   7 8 9 10 11 12
X_Guess = [60 0 0 0   0     0   0 0 0 0  0  0]';

         % e a r thrust
         % 1 2 3 4
U_guess = [0 0 0 1000]';

          % V alpha beta phi theta psi x y z p  q  r  gamma
          % 1 2     3    4   5     6   7 8 9 10 11 12 13
Y_guess = [60 0     0    0   0     0   0 0 0 0  0  0  0]';

Dx_guess = [0 0 0 0 0 0 0 0 0 0 0 0]';

Ix = [2 4 6 10 11 12]';
Iu = [2 3]';
Iy = [1 3 4 6 10 11 12 13]';

Idx = [1 2 3 4 5 6 8 9 10 11 12]';

[x1,u1,y1,dx1] = trim('Cessna_6DOF_trim',X_Guess,U_guess,Y_guess,Ix,Iu,Iy,Dx_guess,Idx);

display(x1)
display(y1)
display(u1)
display(dx1)