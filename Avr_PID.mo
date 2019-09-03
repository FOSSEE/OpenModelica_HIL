model Avr_PID
  inner Modelica_DeviceDrivers.EmbeddedTargets.AVR.Blocks.Microcontroller mcu(platform = Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.Platform.ATmega328P)  annotation(
    Placement(visible = true, transformation(origin = {-76, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI(T = 10 ^ 20, k = 1)  annotation(
    Placement(visible = true, transformation(origin = {-4, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation

  annotation(
    uses(Modelica_DeviceDrivers(version = "1.5.0"), Modelica(version = "3.2.2")));
end Avr_PID;