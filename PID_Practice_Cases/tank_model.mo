model tank_model
 package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

  Modelica.Fluid.Examples.ControlledTankSystem.Utilities.TankController
    tankController(
    waitTime=50,
    maxLevel=0.9*tank1.height,
    minLevel=0.01)
    annotation (Placement(transformation(extent={{-60,-20},{-20,20}})));
  Modelica.Fluid.Examples.ControlledTankSystem.Utilities.RadioButton start(
                                                         reset={stop.on,shut.on},
      buttonTimeTable={20,280})
    annotation (Placement(transformation(extent={{-100,20},{-80,40}})));
  Modelica.Fluid.Examples.ControlledTankSystem.Utilities.RadioButton stop(
                                                        reset={start.on,shut.on},
      buttonTimeTable={220,650})
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  Modelica.Fluid.Examples.ControlledTankSystem.Utilities.RadioButton shut(
                                                        reset={start.on,stop.on},
      buttonTimeTable={700})
    annotation (Placement(transformation(extent={{-100,-40},{-80,-20}})));
  Modelica.Fluid.Valves.ValveDiscrete valve1(                     redeclare
      package Medium = Medium,
    m_flow_nominal=40,
    dp_nominal=100000)
    annotation (Placement(transformation(
        origin={-10,70},
        extent={{10,-10},{-10,10}},
        rotation=180)));
  Modelica.Fluid.Vessels.OpenTank tank1(
    level_start=0.05,
    redeclare package Medium = Medium,
    crossArea=6,
    height=4,
    nPorts=2,
    portsData={Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(
          diameter=0.2,
          height=4,
          zeta_out=0,
          zeta_in=1),Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(
          diameter=0.2,
          height=0,
          zeta_out=0,
          zeta_in=1)})    annotation (Placement(transformation(extent={{10,30},
            {50,70}})));
  Modelica.Blocks.Sources.RealExpression level1(y=tank1.level)
    annotation (Placement(transformation(extent={{-90,-60},{-55,-40}})));
  Modelica.Fluid.Valves.ValveDiscrete valve2(        redeclare package Medium
      = Medium,
    dp_nominal(displayUnit="Pa") = 1,
    m_flow_nominal=100)
    annotation (Placement(transformation(
        origin={34,0},
        extent={{10,-10},{-10,10}},
        rotation=90)));
  Modelica.Fluid.Valves.ValveDiscrete valve3(        redeclare package Medium
      = Medium,
    dp_nominal(displayUnit="Pa") = 1,
    m_flow_nominal=10)
    annotation (Placement(transformation(
        origin={35,-80},
        extent={{10,-10},{-10,10}})));
  Modelica.Fluid.Vessels.OpenTank tank2(
    level_start=0.05,
    redeclare package Medium = Medium,
    height=5,
    crossArea=6,
    nPorts=2,
    portsData={Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(
          diameter=0.2,
          height=5,
          zeta_out=0,
          zeta_in=1),Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(
          diameter=0.2,
          height=0,
          zeta_out=0,
          zeta_in=1)})   annotation (Placement(transformation(extent={{50,-60},
            {90,-20}})));
  Modelica.Fluid.Sources.Boundary_pT ambient1(redeclare package Medium =
        Medium,nPorts=1,
    p=system.p_ambient,
    T=system.T_ambient)
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
  Modelica.Blocks.Sources.RealExpression level2(y=tank2.level)
    annotation (Placement(transformation(extent={{-70,-80},{-33,-60}})));
  Modelica.Fluid.Sources.Boundary_pT source(redeclare package Medium =
        Medium, p=2.5e6,nPorts=1,
    T=system.T_ambient)
    annotation (Placement(transformation(
        origin={-40,70},
        extent={{-10,-10},{10,10}})));
  inner Modelica.Fluid.System system(energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
                        annotation (Placement(transformation(extent={{-90,70},
            {-70,90}})));
equation
  connect(shut.on, tankController.shut) annotation (Line(points={{-79,-30},{
          -72,-30},{-72,-12},{-62,-12}}, color={255,0,255}));
  connect(stop.on, tankController.stop) annotation (Line(points={{-79,0},{-62,
          0}}, color={255,0,255}));
  connect(start.on, tankController.start) annotation (Line(points={{-79,30},{
          -70,30},{-70,12},{-62,12}}, color={255,0,255}));
  connect(tankController.valve1, valve1.open) annotation (Line(points={{-19,12},
          {-10,12},{-10,62}},             color={255,0,255}));
  connect(level1.y, tankController.level1) annotation (Line(points={{-53.25,
          -50},{-52,-50},{-52,-22}}, color={0,0,127}));
  connect(tankController.valve2, valve2.open) annotation (Line(points={{-19,0},
          {-5,0},{26,0}},            color={255,0,255}));
  connect(tankController.valve3, valve3.open) annotation (Line(points={{-19,-12},
          {-10,-12},{-10,-50},{35,-50},{35,-72}},
                                              color={255,0,255}));
  connect(level2.y, tankController.level2) annotation (Line(points={{-31.15,
          -70},{-28,-70},{-28,-22}}, color={0,0,127}));

  connect(source.ports[1], valve1.port_a) annotation (Line(
      points={{-30,70},{-20,70}},
      color={0,127,255}));
  connect(valve3.port_b, ambient1.ports[1]) annotation (Line(
      points={{25,-80},{10,-80}},
      color={0,127,255}));
  connect(tank2.ports[2], valve3.port_a) annotation (Line(
      points={{74,-60},{74,-80},{45,-80}},
      color={0,127,255}));
  connect(valve2.port_b, tank2.ports[1]) annotation (Line(
      points={{34,-10},{34,-20},{50,-20},{50,-60},{66,-60}},
      color={0,127,255}));
  connect(valve1.port_b, tank1.ports[1]) annotation (Line(
      points={{0,70},{10,70},{10,30},{26,30}},
      color={0,127,255}));
  connect(tank1.ports[2], valve2.port_a) annotation (Line(
      points={{34,30},{34,10}},
      color={0,127,255}));
  annotation (
    experiment(StopTime=900),
    Documentation(info="<html>
<p>
With this example, the controller of a tank filling/emptying system
is demonstrated.
</p>

<p>
The basic operation is to fill and empty the two tanks:
</p>
<ol>
<li> Valve 1 is opened and tank 1 is filled.</li>
<li> When tank 1 reaches its fill level limit,
   valve 1 is closed. </li>
<li> After a waiting time, valve 2 is
   opened and the fluid flows from tank 1 into tank 2.</li>
<li> When tank 1 reaches its minimum level, valve 2 is closed. </li>
<li> After a waiting time, valve 3 is opened and
   the fluid flows out of tank 2</li>
<li> When tank 2 reaches its minimum level, valve 3 is closed</li>
</ol>
<p>
The above \"normal\" process can be influenced by three
buttons:
</p>
<ul>
<li> Button <b>start</b> starts the above process.
   When this button is pressed after a \"stop\" or
   \"shut\" operation, the process operation continues.
   </li>
<li> Button <b>stop</b> stops the above process by
   closing all valves. Then, the controller waits for
   further input (either \"start\" or \"shut\" operation).</li>
<li> Button <b>shut</b> is used to shutdown the process,
   by emptying at once both tanks by opening valve 2 and
   valve 3. When this is achieved,
   the process goes back to its start configuration
   where all 3 valves are closed.
   Clicking on \"start\", restarts the process.</li>
</ul>

<p>
The demo-run uses the following button presses:
</p>

<ul>
<li> Button <b>start</b> pressed at 20 s.</li>
<li> Button <b>stop</b> pressed at 220 s </li>
<li> Button <b>start</b> pressed at 280 s </li>
<li> Button <b>stop</b> pressed at 650 s </li>
<li> Button <b>shut</b> pressed at 700 s </li>
<li> Simulate for 900 s</li>
</ul>

<p>
This example is based on
</p>

<dl>
<dt>Dressler I. (2004):</dt>
<dd> <b>Code Generation From JGrafchart to Modelica</b>.
   Master thesis, supervisor: Karl-Erik Arzen,
   Department of Automatic Control, Lund Institute of Technology,
   Lund, Sweden, March 30, 2004<br>&nbsp;</dd>
</dl>

<img src=\"modelica://Modelica/Resources/Images/Fluid/Examples/ControlledTanks.png\" border=\"1\"
   alt=\"ControlledTanks.png\">
</html>"),
    __Dymola_Commands(file=
          "modelica://Modelica/Resources/Scripts/Dymola/Fluid/ControlledTanks/plot level and ports.m_flow.mos"
        "plot level and ports.m_flow"));
end tank_model;