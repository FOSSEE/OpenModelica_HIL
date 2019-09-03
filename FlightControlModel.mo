model FlightControlModel
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {26, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback2 annotation(
    Placement(visible = true, transformation(origin = {-52, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FlightLongs flightLongs1 annotation(
    Placement(visible = true, transformation(origin = {72, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = 0.005)  annotation(
    Placement(visible = true, transformation(origin = {53, -17}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI(T = 15, k = 0.2)  annotation(
    Placement(visible = true, transformation(origin = {-14, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant altitude(k = 100)  annotation(
    Placement(visible = true, transformation(origin = {-148, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback3 annotation(
    Placement(visible = true, transformation(origin = {-114, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI1(T = 20, k = 0.1) annotation(
    Placement(visible = true, transformation(origin = {-84, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant thrust(k = 1112.82) annotation(
    Placement(visible = true, transformation(origin = {22, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(thrust.y, flightLongs1.thrust) annotation(
    Line(points = {{34, 54}, {46, 54}, {46, 20}, {62, 20}, {62, 20}}, color = {0, 0, 127}));
  connect(flightLongs1.theta, feedback2.u2) annotation(
    Line(points = {{84, 16}, {94, 16}, {94, -28}, {-52, -28}, {-52, 4}, {-52, 4}}, color = {0, 0, 127}));
  connect(flightLongs1.q, gain2.u) annotation(
    Line(points = {{84, 12}, {90, 12}, {90, -18}, {60, -18}, {60, -16}}, color = {0, 0, 127}));
  connect(feedback1.y, flightLongs1.del) annotation(
    Line(points = {{36, 12}, {60, 12}, {60, 12}, {62, 12}}, color = {0, 0, 127}));
  connect(flightLongs1.z, feedback3.u2) annotation(
    Line(points = {{83, 21}, {104, 21}, {104, -38}, {-112, -38}, {-112, 4}, {-114, 4}}, color = {0, 0, 127}));
  connect(PI1.y, feedback2.u1) annotation(
    Line(points = {{-72, 12}, {-62, 12}, {-62, 12}, {-60, 12}}, color = {0, 0, 127}));
  connect(feedback3.y, PI1.u) annotation(
    Line(points = {{-104, 12}, {-96, 12}, {-96, 12}, {-96, 12}}, color = {0, 0, 127}));
  connect(feedback3.u1, altitude.y) annotation(
    Line(points = {{-122, 12}, {-136, 12}, {-136, 12}, {-136, 12}}, color = {0, 0, 127}));
  connect(PI.y, feedback1.u1) annotation(
    Line(points = {{-3, 12}, {18, 12}}, color = {0, 0, 127}));
  connect(PI.u, feedback2.y) annotation(
    Line(points = {{-26, 12}, {-42, 12}}, color = {0, 0, 127}));
  connect(gain2.y, feedback1.u2) annotation(
    Line(points = {{47.5, -17}, {26, -17}, {26, -6.5}, {26, -6.5}, {26, 4}}, color = {0, 0, 127}));
  annotation(experiment(StartTime = 0, StopTime = 500, Interval = 0.002),    uses(Modelica(version = "3.2.2")));

    end FlightControlModel;