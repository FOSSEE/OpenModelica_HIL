model FlightPitchHoldHIL
  FlightLongs flightLongs1 annotation(
    Placement(visible = true, transformation(origin = {68, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Real ModelicaInput (start = 0, fixed = true );

  Real ModelicaOutput1 (fixed = true );
  Real ModelicaOutput2 (fixed = true );

  Real OutputDummy1;
  Real OutputDummy2;
 parameter Real sampleTime = 0.02;
  Modelica.Blocks.Sources.Constant const(k = 1112.82)  annotation(
    Placement(visible = true, transformation(origin = {24, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(const.y, flightLongs1.thrust) annotation(
    Line(points = {{36, 48}, {44, 48}, {44, 20}, {58, 20}, {58, 20}}, color = {0, 0, 127}));
ModelicaOutput1 = flightLongs1.q;
ModelicaOutput2 = flightLongs1.theta;
flightLongs1.del = ModelicaInput ;  
when sample(0, 0.018) then
    ModelicaInput = InterProcessCommunication.SharedMemory.SharedMemoryRead(1)         "SharedMemoryRead Function reads the value from the shared memory, pointed by pidOutputIndex tag and assigns it to the input of the DC motor";
    OutputDummy1 = InterProcessCommunication.SharedMemory.SharedMemoryWrite(1, ModelicaOutput1)
 "SharedMemoryWrite Function writes the value of measured speed into the shared memory, pointed by pidInputIndex tag" ;
    OutputDummy2 = InterProcessCommunication.SharedMemory.SharedMemoryWrite(2, ModelicaOutput2);

  end when;
//equations from serial_read
  annotation(
    uses(Modelica(version = "3.2.2")),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -100}, {140, 100}}, initialScale = 0.1), graphics = {Text(lineColor = {255, 0, 0}, extent = {{40, 37}, {90, 31}}, textString = "plant"), Rectangle(origin = {-24, 28},lineColor = {255, 0, 0}, extent = {{32, 40}, {110, -38}})}),
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2})),
    experiment(StopTime = 100, StartTime = 0, Tolerance = 1e-06, Interval = 0.01),
    __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS", nls = "homotopy", clock = "RT"));

end FlightPitchHoldHIL;