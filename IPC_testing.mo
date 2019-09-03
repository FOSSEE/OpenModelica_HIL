class IPC_testing
  import InterProcessCommunication.SharedMemory.*;
  //extends Modelica.Mechanics.Rotational.Components;
  // Declaration of variables and constants
  Real motorInputValue "Value of input to the  Discrete PID Controller";
  Real motorOutputValue "Value of output of Discrete PID Controller";
  Integer motorInputIndex = 1;
  //Address from where to read, can be any number between 0 to 10, it must match the address given to output value in second model i.e. DiscretePID_SM_Example in this case
  Integer motorOutputIndex = 1;
  //Address where to write, can be any number between 0 to 10
  Real motorOutputDummy "Dummy value to be returned by the SharedMemoryWrite function";
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 1, k = 1)  annotation(
    Placement(visible = true, transformation(origin = {-50, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  firstOrder1.u = motorInputValue "The value of control signal (pidOutputValue) is acquired by the SharedMemoryRead function and it is assigned to the input of the DC motor";
  motorOutputValue = firstOrder1.y "The measured speed of the DC motor is assigned to pidInputValue, which is written into the shared memory using SharedMemoryWrite Function";
  when sample(0, 0.02) then
    motorInputValue = InterProcessCommunication.SharedMemory.SharedMemoryRead(motorInputIndex) "SharedMemoryRead Function reads the value from the shared memory, pointed by pidOutputIndex tag and assigns it to the input of the DC motor";
    motorOutputDummy = InterProcessCommunication.SharedMemory.SharedMemoryWrite(motorOutputIndex, motorOutputValue) "SharedMemoryWrite Function writes the value of measured speed into the shared memory, pointed by pidInputIndex tag";
  end when;
  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -100}, {140, 100}}, initialScale = 0.1)),
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, grid = {2, 2})),
    Documentation(info = "<html>

<p>
<b>Inter Process Communication Library V1.0</b><br /><br />
The <b>DCMotor</b> model contains the DC motor, which reads the control signal from shared memory (given by discrete PID controller) and the speed sensor measures the resulting rotation. The measured speed from speed sensor is written into the shared memory.
 
 </p>
 
 <p>
The values of control signal and speed of the DC motor are read from and written into the shared memory at sampling interval of 0.05 seconds.  <a href=\"modelica://InterProcessCommunication.SharedMemory.SharedMemoryRead\"> SharedMemoryRead </a> and <a href=\"modelica://InterProcessCommunication.SharedMemory.SharedMemoryWrite\"> SharedMemoryWrite </a> functions are used to read the control signal from the shared memory and write the measured speed into the shared memory respectively. The motorOutputIndex and motorInputIndex variables serve as index of the tag i.e. motorInputIndex points to the value of control signal and motorOutputIndex points to the value of speed of the DC motor. The value of the control signal, returned by the <a href=\"modelica://InterProcessCommunication.SharedMemory.SharedMemoryRead\"> SharedMemoryRead </a> function is stored in the motorInputValue variable. The value of the motorInputValue is in turn assigned to the torque.tau variable, which serves as an input to the DC motor. Similarly, the value of the measured speed is  assigned to motorOutputvalue variable. Therefore, the value of the motorOutputvalue is written into shared memory using <a href=\"modelica://InterProcessCommunication.SharedMemory.SharedMemoryWrite\"> SharedMemoryWrite </a> function. 
</p>

<p>
<b>License:</b> GNU GPLV3 2017<br />
This is a free program, and you are welcome to modify and redistribute it. This program comes with ABSOLUTELY NO WARRANTY.<br /><br />
<b>Credits:</b> ModeliCon Infotech Team <br />Ankur Gajjar <br />Shubham Patne <br />Jal Panchal <br />Ritesh Sharma <br />Pavan P <br /> 
</p>

</html>"),
    experiment(StopTime = 30, StartTime = 0, Tolerance = 1e-06, Interval = 0.01),
    __OpenModelica_simulationFlags(jacobian = "coloredNumerical", s = "dassl", lv = "LOG_STATS", nls = "homotopy", clock = "RT"),
    uses(Modelica(version = "3.2.2")));
end IPC_testing;