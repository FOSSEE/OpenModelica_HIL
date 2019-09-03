class HIL_PID
  import Modelica_DeviceDrivers.*;
  Modelica_DeviceDrivers.Blocks.Communication.SerialPortSend serialSend(Serial_Port = "/dev/ttyACM0", autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, sampleTime = 0.002) annotation(
    Placement(visible = true, transformation(origin = {-46, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.Packager packager(sampleTime = 0.001, useBackwardPropagatedBufferSize = true, useBackwardSampleTimePropagation = true) annotation(
    Placement(visible = true, transformation(extent = {{-56, 18}, {-36, 38}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Communication.SerialPortReceive serialReceive(Serial_Port = "/dev/ttyACM0", autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, sampleTime = 0.002, startTime = 0) annotation(
    Placement(visible = true, transformation(origin = {-8, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.UnpackUnsignedInteger unpackInt(nu = 1, width = 8) annotation(
    Placement(visible = true, transformation(extent = {{-18, -2}, {2, 18}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.AddInteger addInteger(n = 1, nu = 1) annotation(
    Placement(visible = true, transformation(extent = {{-56, -34}, {-36, -14}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetInteger getInteger annotation(
    Placement(visible = true, transformation(extent = {{-20, -78}, {0, -58}}, rotation = 0)));
  Modelica.Blocks.Math.RealToInteger realToInteger1 annotation(
    Placement(visible = true, transformation(origin = {-76, 62}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Math.IntegerToReal integerToReal1 annotation(
    Placement(visible = true, transformation(origin = {28, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 1, initType = Modelica.Blocks.Types.Init.NoInit, k = 1) annotation(
    Placement(visible = true, transformation(origin = {-26, 86}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime1 annotation(
    Placement(visible = true, transformation(origin = {70, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(addInteger.pkgOut[1], serialSend.pkgIn) annotation(
    Line(points = {{-46, -35}, {-46, -79}}));
  connect(realToInteger1.y, addInteger.u[1]) annotation(
    Line(points = {{-76, 51}, {-76, -24}, {-58, -24}}, color = {255, 127, 0}));
  connect(realToInteger1.u, firstOrder1.y) annotation(
    Line(points = {{-76, 74}, {-76, 86}, {-38, 86}}, color = {0, 0, 127}));
  connect(packager.pkgOut, addInteger.pkgIn) annotation(
    Line(points = {{-46, 17}, {-46, -13}}));
  connect(unpackInt.pkgOut[1], getInteger.pkgIn) annotation(
    Line(points = {{-8, -3}, {-7, -3}, {-7, -20}, {-10, -20}, {-10, -57}}, thickness = 0.5));
  connect(serialReceive.pkgOut, unpackInt.pkgIn) annotation(
    Line(points = {{-8, 39}, {-8, 19}}));
  connect(integerToReal1.u, unpackInt.y) annotation(
    Line(points = {{28, 58}, {28, 8}, {3, 8}}, color = {255, 127, 0}));
  connect(firstOrder1.u, integerToReal1.y) annotation(
    Line(points = {{-14, 86}, {28, 86}, {28, 82}, {28, 82}, {28, 82}}, color = {0, 0, 127}));
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
    uses(Modelica_DeviceDrivers(version = "1.5.0")));
end HIL_PID;