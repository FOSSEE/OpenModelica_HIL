class Serial_read_write
    import OpenModelica.Scripting.*;
  import Modelica_DeviceDrivers;
  Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime annotation(
    Placement(transformation(extent = {{62, 50}, {82, 70}})));
  Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
    Placement(visible = true, transformation(origin = {-68, 26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddInteger addInteger1(nu = 1)  annotation(
    Placement(visible = true, transformation(origin = {-36, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine1(amplitude = 100, freqHz = 0.5, offset = 100, startTime = 0) annotation(
    Placement(visible = true, transformation(origin = {-68, 66}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.Packager packager(sampleTime = 0.005, useBackwardPropagatedBufferSize = true, useBackwardSampleTimePropagation = false) annotation(
    Placement(visible = true, transformation(extent = {{-44, 30}, {-24, 50}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Communication.SerialPortSend serialSend(Serial_Port = "/dev/ttyACM0", autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, sampleTime = 0.002) annotation(
    Placement(visible = true, transformation(origin = {-36, -58}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica_DeviceDrivers.Blocks.Communication.SerialPortReceive serialReceive(Serial_Port = "/dev/ttyACM0", autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, enableExternalTrigger = false, sampleTime = 0.002)  annotation(
    Placement(visible = true, transformation(origin = {18, 54}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetInteger getInteger annotation(
    Placement(visible = true, transformation(extent = {{6, -66}, {26, -46}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.UnpackUnsignedInteger unpackInt(nu = 1, width = 8)  annotation(
    Placement(visible = true, transformation(origin = {16, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
alarm(1);
  connect(unpackInt.pkgOut[1], getInteger.pkgIn) annotation(
    Line(points = {{16, -6}, {18, -6}, {18, -46}, {16, -46}}, thickness = 0.5));
  connect(serialReceive.pkgOut, unpackInt.pkgIn) annotation(
    Line(points = {{18, 44}, {16, 44}, {16, 14}, {16, 14}, {16, 14}}));
  connect(packager.pkgOut, addInteger1.pkgIn) annotation(
    Line(points = {{-34, 29}, {-34, 17.5}, {-36, 17.5}, {-36, 6}}));
  connect(addInteger1.pkgOut[1], serialSend.pkgIn) annotation(
    Line(points = {{-36, -14}, {-36, -47}}, thickness = 0.5));
  connect(sine1.y, realToInteger1.u) annotation(
    Line(points = {{-68, 55}, {-68, 38}}, color = {0, 0, 127}));
  connect(realToInteger1.y, addInteger1.u[1]) annotation(
    Line(points = {{-68, 16}, {-68, 16}, {-68, -4}, {-48, -4}, {-48, -4}}, color = {255, 127, 0}));
  annotation(
    Documentation(info = "<html>
<h4><span style=\"color:#008000\">Example for serial port support</span></h4>
<h4><span style=\"color:#008000\">Hardware setup</span></h4>
<p>In order to execute the example an appropriate physical connection between the sending and the receiving serial port needs to be established, (e.g., by using a null modem cable between the two serial port interfaces <a href=\"http://en.wikipedia.org/wiki/Null_modem\">http://en.wikipedia.org/wiki/Null_modem</a>). In fact a minimal mull modem with lines (<code>TxD</code>, <code>Rxd</code> and <code>GND</code>) is sufficient. Next, the <code>SerialPortReceive</code> and <code>SerialPortSend</code> blocks parameters must be updated with the device filenames corresponding to the connected physical serial ports. Now, the example can be executed.</p>
<h4><span style=\"color:#008000\">Alternative: Using virtual serial port devices for test purposes</span></h4>
<p>To run the example without serial port hardware, it is possible to resort to virtual serial ports. Possible ways of doing this are described in the following.</p>
<p>On Linux, make sure that <i>socat</i> is installed, e.g., on an Ubuntu machine do</p>
<pre>sudo aptitude install socat</pre>
<p>Now open a console and create two virtual serial port interfaces using socat:</p>
<pre>socat -d -d pty,raw,echo=0 pty,raw,echo=0</pre>
<p>The socat program will print the device file names that it created. The output will resemble the following:</p>
<pre>2013/11/24 15:20:21 socat[3262] N PTY is /dev/pts/1
2013/11/24 15:20:21 socat[3262] N PTY is /dev/pts/3
2013/11/24 15:20:21 socat[3262] N starting data transfer loop with FDs [3,3] and [5,5]</pre>
<p>Use them in the Send and Receive block. E.g., for the output above you would use <code>&quot;/dev/pts/1&quot;</code> in <code>SerialPortReceive</code> and <code>&quot;/dev/pts/3&quot;</code> in <code>SerialPortSend</code>.</p>
<p>You may have also have a look at the discussion about virtual serial port devices on stackoverflow<a href=\"http://stackoverflow.com/questions/52187/virtual-serial-port-for-linux\">http://stackoverflow.com/questions/52187/virtual-serial-port-for-linux</a>.</p>
<p>On Windows, make sure that the null modem emulator <i>com0com</i> is installed.</p>
<p>Start the Setup for <i>com0com</i> and check the device names of the created virtual port pair. E.g. you could type <code>&quot;COM6&quot;</code> in <code>SerialPortReceive</code> and <code>&quot;COM7&quot;</code> in <code>SerialPortSend</code>.</p>
</html>"),
    uses(Modelica(version = "3.2.2"), Modelica_DeviceDrivers(version = "1.5.0")));

end Serial_read_write;