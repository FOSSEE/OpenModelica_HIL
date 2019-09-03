[A, B, C, D] = linmod('Cessna_6DOF_Wind_trim'); % V gamma q alpha

A_lon = [A(1,1) A(1,8) A(1,11) A(1,2);
    A(8,1) A(8,8) A(8,11) A(8,2);
    A(11,1) A(11,8) A(11,11) A(11,2);
    A(2,1) A(2,8) A(2,11) A(2,2)];

A_phug = [A(1,1) A(1,8);
    A(8,1) A(8,8)];

A_short = [A(11,11) A(11,2);
    A(2,11) A(2,2)];

%display(eig(A_lon))
%display(eig(A_phug))
%display(eig(A_short))