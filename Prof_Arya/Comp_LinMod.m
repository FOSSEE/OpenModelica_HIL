[A, B, C, D] = linmod('Cessna_6DOF_trim');

A = [A(1,1) A(1,3) A(1,5) A(1,11);
    A(3,1) A(3,3) A(3,5) A(3,11);
    A(5,1) A(5,3) A(5,5) A(5,11);
    A(11,1) A(11,3) A(11,5) A(11,11)];

B = [B(1,1) B(1,4);
    B(3,1) B(3,4);
    B(5,1) B(5,4);
    B(11,1) B(11,4)];

X(1:4,1) = [u, w, theta, q];

for kk = 1:10/0.001
    
    X(:,kk+1) = X(:,kk) + (A*(X(:,kk) - X(:,1)) + B*[(U(kk,1) + 0.0303) 0]')*0.001;
    
end

plot(X(1,:))
hold on
plot(V_arr.*cos(alpha_arr),'--')
legend('Linearized','Nonlinear')
xlabel('u (m/s)')

figure
plot(X(2,:))
hold on
plot(alpha_arr*180/pi,'--')
legend('Linearized','Nonlinear')
xlabel('alpha (degree)')