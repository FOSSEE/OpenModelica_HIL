model RLC_ckt
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R = 10)  annotation(
    Placement(visible = true, transformation(origin = {-14, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor1 annotation(
    Placement(visible = true, transformation(origin = {24, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L = 1)  annotation(
    Placement(visible = true, transformation(origin = {54, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
    Placement(visible = true, transformation(origin = {26, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage1 annotation(
    Placement(visible = true, transformation(origin = {-44, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
    Placement(visible = true, transformation(origin = {22, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PID PID(Td = 1, Ti = 0.5, k = 1)  annotation(
    Placement(visible = true, transformation(origin = {-72, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-100, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Step step1(startTime = 50)  annotation(
    Placement(visible = true, transformation(origin = {-128, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(feedback1.u1, step1.y) annotation(
    Line(points = {{-108, 62}, {-116, 62}, {-116, 62}, {-116, 62}}, color = {0, 0, 127}));
  connect(voltageSensor1.v, feedback1.u2) annotation(
    Line(points = {{22, -26}, {-96, -26}, {-96, 52}, {-100, 52}, {-100, 54}}, color = {0, 0, 127}));
  connect(PID.y, signalVoltage1.v) annotation(
    Line(points = {{-60, 62}, {-44, 62}, {-44, 42}, {-44, 42}}, color = {0, 0, 127}));
  connect(feedback1.y, PID.u) annotation(
    Line(points = {{-90, 62}, {-84, 62}, {-84, 62}, {-84, 62}}, color = {0, 0, 127}));
  connect(voltageSensor1.n, ground1.p) annotation(
    Line(points = {{32, -16}, {26, -16}, {26, -48}, {26, -48}}, color = {0, 0, 255}));
  connect(voltageSensor1.p, signalVoltage1.p) annotation(
    Line(points = {{12, -16}, {-74, -16}, {-74, 32}, {-54, 32}, {-54, 34}, {-54, 34}}, color = {0, 0, 255}));
  connect(inductor1.n, voltageSensor1.n) annotation(
    Line(points = {{64, 34}, {74, 34}, {74, -14}, {32, -14}, {32, -16}, {32, -16}}, color = {0, 0, 255}));
  connect(resistor1.p, signalVoltage1.n) annotation(
    Line(points = {{-24, 34}, {-34, 34}, {-34, 34}, {-34, 34}}, color = {0, 0, 255}));
  connect(capacitor1.n, inductor1.p) annotation(
    Line(points = {{34, 34}, {44, 34}, {44, 34}, {44, 34}, {44, 34}, {44, 34}}, color = {0, 0, 255}));
  connect(resistor1.n, capacitor1.p) annotation(
    Line(points = {{-4, 34}, {14, 34}, {14, 34}, {14, 34}}, color = {0, 0, 255}));

annotation(
    uses(Modelica(version = "3.2.2")));end RLC_ckt;