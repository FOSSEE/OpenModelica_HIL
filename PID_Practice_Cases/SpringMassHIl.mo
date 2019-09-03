model SpringMassHIl
import InterProcessCommunication.SharedMemory.*;
  Modelica.Mechanics.Translational.Components.Mass mass1(L = 1, m = 1)  annotation(
    Placement(visible = true, transformation(origin = {36, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sources.Force force1 annotation(
    Placement(visible = true, transformation(origin = {6, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Fixed fixed1 annotation(
    Placement(visible = true, transformation(origin = {94, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y annotation(
    Placement(visible = true, transformation(origin = {-28, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 
   // Declaration of variables and constants
  Real motorInputValue "Value of input to the  Discrete PID Controller" ;
  Real motorOutputValue  "Value of output of Discrete PID Controller" ;
  Integer motorInputIndex = 1;
  Real ReferenceOp "Reference Output";
  //Address from where to read, can be any number between 0 to 10, it must match the address given to output value in second model i.e. DiscretePID_SM_Example in this case
  Integer motorOutputIndex = 1;//Address where to write, can be any number between 0 to 10
  Real motorOutputDummy "Dummy value to be returned by the SharedMemoryWrite function";
 //Functions from serial_read
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.UnpackUnsignedInteger unpackInt(nu = 1, width = 8) annotation(
    Placement(visible = true, transformation(extent = {{-90, -6}, {-70, 14}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetInteger getInteger annotation(
    Placement(visible = true, transformation(extent = {{-90, -62}, {-70, -42}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Communication.SerialPortReceive serialReceive(Serial_Port = "/dev/ttyACM1", autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, enableExternalTrigger = false, sampleTime = 0.01)  annotation(
    Placement(visible = true, transformation(origin = {-80, 58}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
 Modelica.Mechanics.Translational.Components.SpringDamper springDamper1(c = 100, d = 10, s_rel0 = 1)  annotation(
    Placement(visible = true, transformation(origin = {66, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor1 annotation(
    Placement(visible = true, transformation(origin = {16, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
equation
  connect(mass1.flange_b, positionSensor1.flange) annotation(
    Line(points = {{46, 4}, {48, 4}, {48, -36}, {26, -36}, {26, -36}}, color = {0, 127, 0}));
  connect(positionSensor1.s, y) annotation(
    Line(points = {{4, -36}, {-20, -36}, {-20, -36}, {-28, -36}}, color = {0, 0, 127}));
  connect(springDamper1.flange_b, fixed1.flange) annotation(
    Line(points = {{76, 4}, {92, 4}, {92, 4}, {94, 4}}, color = {0, 127, 0}));
  connect(mass1.flange_b, springDamper1.flange_a) annotation(
    Line(points = {{46, 4}, {56, 4}, {56, 4}, {56, 4}}, color = {0, 127, 0}));
  connect(unpackInt.pkgOut[1], getInteger.pkgIn) annotation(
    Line(points = {{-80, -7}, {-80, -41}}, thickness = 0.5));
  connect(serialReceive.pkgOut, unpackInt.pkgIn) annotation(
    Line(points = {{-80, 47}, {-80, 15}}));
  connect(force1.flange, mass1.flange_a) annotation(
    Line(points = {{16, 4}, {26, 4}, {26, 4}, {26, 4}}, color = {0, 127, 0}));
  force1.f = motorInputValue "The value of control signal (pidOutputValue) is acquired by the SharedMemoryRead function and it is assigned to the input of the system";
  motorOutputValue = y "The measured speed of the system is assigned to pidInputValue, which is written into the shared memory using SharedMemoryWrite Function";
  when sample(0, 0.05) then
    motorInputValue = InterProcessCommunication.SharedMemory.SharedMemoryRead(motorInputIndex) "SharedMemoryRead Function reads the value from the shared memory, pointed by pidOutputIndex tag and assigns it to the input of the system";
    motorOutputDummy = InterProcessCommunication.SharedMemory.SharedMemoryWrite(motorOutputIndex, motorOutputValue) "SharedMemoryWrite Function writes the value of measured speed into the shared memory, pointed by pidInputIndex tag";
  end when;
  ReferenceOp= unpackInt.y;
//equations from serial_read
  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -100}, {140, 100}}, initialScale = 0.1), graphics = {Text(lineColor = {255, 0, 0}, extent = {{40, 37}, {90, 31}}, textString = "plant"), Rectangle(origin = {-12, -2},lineColor = {255, 0, 0}, extent = {{0, 68}, {140, -54}})}),
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2})),
    Documentation(info = "<html>

<p>
<b>Inter Process Communication Library V1.0</b><br /><br />
The <b>DCMotor</b> model contains the DC motor, which reads the control signal from shared memory (given by discrete PID controller) and the speed sensor measures the resulting rotation. The measured speed from speed sensor is written into the shared memory.
 
 </p>
 
 <p>
The values of control signal and speed of the spring mass system are read from and written into the shared memory at sampling interval of 0.05 seconds.  <a href=\"modelica://InterProcessCommunication.SharedMemory.SharedMemoryRead\"> SharedMemoryRead </a> and <a href=\"modelica://InterProcessCommunication.SharedMemory.SharedMemoryWrite\"> SharedMemoryWrite </a> functions are used to read the control signal from the shared memory and write the measured speed into the shared memory respectively. The motorOutputIndex and motorInputIndex variables serve as index of the tag i.e. motorInputIndex points to the value of control signal and motorOutputIndex points to the value of speed of the DC motor. The value of the control signal, returned by the <a href=\"modelica://InterProcessCommunication.SharedMemory.SharedMemoryRead\"> SharedMemoryRead </a> function is stored in the motorInputValue variable. The value of the motorInputValue is in turn assigned to the force.u variable, which serves as an input to the spring-mass system. Similarly, the value of the measured speed is  assigned to motorOutputvalue variable. Therefore, the value of the motorOutputvalue is written into shared memory using <a href=\"modelica://InterProcessCommunication.SharedMemory.SharedMemoryWrite\"> SharedMemoryWrite </a> function. 
</p>


</html>"),
    experiment(StopTime = 60, StartTime = 0, Tolerance = 1e-06, Interval = 0.01),
    __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS", nls = "homotopy", clock = "RT"),
    uses(Modelica(version = "3.2.2")));

end SpringMassHIl;