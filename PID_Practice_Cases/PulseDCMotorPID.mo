model PulseDCMotorPID"PID and Motor combine model with Pulse input"
 Modelica.Blocks.Continuous.PID PID( Td = 0, Ti = 10 ^ 20, k = 15)  annotation(
    Placement(visible = true, transformation(origin = {-30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-60, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque torque1 annotation(
    Placement(visible = true, transformation(origin = {30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 5)  annotation(
    Placement(visible = true, transformation(origin = {70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor1 annotation(
    Placement(visible = true, transformation(origin = {90, -28}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
 Modelica.Blocks.Sources.Pulse pulse1(amplitude = 833, offset = 0, period = 20, startTime = 0) annotation(
    Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(pulse1.y,feedback1.u1) annotation(
    Line(points = {{-78, 10}, {-68, 10}}, color = {0, 0, 127}));
  connect(PID.y, torque1.tau) annotation(
    Line(points = {{-18, 10}, {16, 10}, {16, 10}, {18, 10}}, color = {0, 0, 127}));
  connect(inertia1.flange_b, speedSensor1.flange) annotation(
    Line(points = {{80, 10}, {90, 10}, {90, -18}, {90, -18}}));
  connect(feedback1.u2, speedSensor1.w) annotation(
    Line(points = {{-60, 2}, {-60, -46}, {90, -46}, {90, -38}}, color = {0, 0, 127}));
  connect(feedback1.y, PID.u) annotation(
    Line(points = {{-51, 10}, {-42, 10}}, color = {0, 0, 127}));
  connect(torque1.flange, inertia1.flange_a) annotation(
    Line(points = {{40, 10}, {60, 10}, {60, 10}, {60, 10}}));
  annotation (Documentation(info= "<html>
<p>
<b>Inter Process Communication Library V1.0</b><br /><br />
This is a combined model for PID and Motor model with square pulse input.
</p>
<p>
<b>License:</b> OSMC-PL v1.2 2017<br /><br />
</p>
</html>"));

end PulseDCMotorPID;