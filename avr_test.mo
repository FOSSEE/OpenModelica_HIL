within MDD_Practice;
model avr_test

  Modelica.Blocks.Sources.Sine sine1(amplitude = 100, freqHz = 0.25, offset = 50)  annotation(
    Placement(visible = true, transformation(origin = {-72, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.Packager packager1(useBackwardPropagatedBufferSize = true, useBackwardSampleTimePropagation = true, userBufferSize = 16 * 64)  annotation(
    Placement(visible = true, transformation(origin = {-30, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddReal addReal1(byteOrder = Modelica_DeviceDrivers.Utilities.Types.ByteOrder.LE, n = 1, nu = 1)  annotation(
    Placement(visible = true, transformation(origin = {-30, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Communication.SerialPortSend serialPortSend1(Serial_Port = "/dev/ttyACM1",autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B9600, enableExternalTrigger = false, sampleTime = 0.1)  annotation(
    Placement(visible = true, transformation(origin = {-30, 4}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
    Placement(visible = true, transformation(origin = {68, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Communication.SerialPortReceive serialPortReceive1(Serial_Port = "/dev/ttyACM1", autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B9600)  annotation(
    Placement(visible = true, transformation(origin = {32, 56}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetReal getReal1 annotation(
    Placement(visible = true, transformation(origin = {32,16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(serialPortReceive1.pkgOut, getReal1.pkgIn) annotation(
    Line(points = {{32, 45}, {32, 27}}));
  connect(addReal1.pkgOut[1], serialPortSend1.pkgIn) annotation(
    Line(points = {{-30, 29.2}, {-30, 29.2}, {-30, 13.2}, {-30, 13.2}}, thickness = 0.5));
  connect(sine1.y, addReal1.u[1]) annotation(
    Line(points = {{-61, 40}, {-43, 40}, {-43, 40}, {-43, 40}}, color = {0, 0, 127}));
  connect(packager1.pkgOut, addReal1.pkgIn) annotation(
    Line(points = {{-30, 65.2}, {-30, 65.2}, {-30, 49.2}, {-30, 49.2}}));
  annotation(
    uses(Modelica_DeviceDrivers(version = "1.5.0"), Modelica(version = "3.2.2")));
end avr_test;