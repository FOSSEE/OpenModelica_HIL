class ArduinoIPCWrite

import InterProcessCommunication.SharedMemory.*;
Real ModelicaInput;
  Real ModelicaOutput (start = 1);
  Real OutputDummy;  
  equation
    
   
    ModelicaOutput = sin(time)*100;
    when sample(0, 0.005) then
     
      ModelicaInput = InterProcessCommunication.SharedMemory.SharedMemoryRead(1);
      OutputDummy = InterProcessCommunication.SharedMemory.SharedMemoryWrite(1, ModelicaOutput);   
      
    end when;
annotation(
    experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-6, Interval = 0.12));
end ArduinoIPCWrite;