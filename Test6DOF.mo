model Test6DOF
  Real F[3] = {0, -1,9.8};
  Real M[3] = {0 , 0, 0};
  Flight6DOF flight6DOF1(angles(start = {0, 0, 0}),omega(start = {0, 0, 1}), pos(start = {0, -1, 0}), v(start = {1, 0, 0}))  annotation(
    Placement(visible = true, transformation(origin = {6, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  flight6DOF1.Force = F;
  flight6DOF1.Moment = M;
end Test6DOF;