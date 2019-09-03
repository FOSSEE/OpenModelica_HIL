class Serial_read
  Modelica_DeviceDrivers.Blocks.OperatingSystem.SynchronizeRealtime synchronizeRealtime annotation(
    Placement(transformation(extent = {{62, 50}, {82, 70}})));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.UnpackUnsignedInteger unpackInt(nu = 1, width = 8) annotation(
    Placement(visible = true, transformation(extent = {{4, -4}, {24, 16}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Packaging.SerialPackager.GetInteger getInteger annotation(
    Placement(visible = true, transformation(extent = {{4, -66}, {24, -46}}, rotation = 0)));
  Modelica_DeviceDrivers.Blocks.Communication.SerialPortReceive serialReceive(Serial_Port = "/dev/ttyACM1", autoBufferSize = true, baud = Modelica_DeviceDrivers.Utilities.Types.SerialBaudRate.B115200, enableExternalTrigger = false, sampleTime = 0.002)  annotation(
    Placement(visible = true, transformation(origin = {14, 54}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
equation
  connect(serialReceive.pkgOut, unpackInt.pkgIn) annotation(
    Line(points = {{14, 43}, {14, 17}}));
  connect(unpackInt.pkgOut[1], getInteger.pkgIn) annotation(
    Line(points = {{14, -5}, {14, -45}}, thickness = 0.5));
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
</html>"));

end Serial_read;