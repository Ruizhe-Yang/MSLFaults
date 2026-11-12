within MSLFaults.Utilities.Internal;
model ValveCircuit "Compare isenthalpic two-port paths and a zero-flow pressure reading"
  replaceable package Medium=Modelica.Media.Water.ConstantPropertyLiquidWater constrainedby Modelica.Media.Interfaces.PartialMedium;
  inner Modelica.Fluid.System system;
  parameter Boolean reverse=false;
  parameter Boolean varyOpening=false;
  Modelica.Fluid.Sources.Boundary_pT upstream(redeclare package Medium=Medium,nPorts=2,p=if reverse then 1e5 else 2e5,T=306) annotation(Placement(transformation(origin={-165,55},extent={{-17,-17},{17,17}},rotation=0)));
  Modelica.Fluid.Sources.Boundary_pT downstream(redeclare package Medium=Medium,nPorts=1,p=if reverse then 2e5 else 1e5,T=296) annotation(Placement(transformation(origin={164,55},extent={{-17,-17},{17,17}},rotation=180)));
  replaceable Modelica.Fluid.Valves.ValveLinear valve annotation(Placement(transformation(origin={4,55},extent={{-17,-17},{17,17}},rotation=0))) constrainedby Modelica.Fluid.Valves.ValveLinear(redeclare package Medium=Medium,m_flow_nominal=1,dp_nominal=1e5);
  Modelica.Blocks.Sources.RealExpression valveCommand(y=if varyOpening then min(1,0.25+0.25*time) else 1) annotation(Placement(transformation(origin={4,121},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Fluid.Sensors.Pressure sensor annotation(Placement(transformation(origin={-109,-46},extent={{-17,-17},{17,17}},rotation=0))) constrainedby Modelica.Fluid.Sensors.Pressure(redeclare package Medium=Medium);
  Modelica.Blocks.Sources.RealExpression massFlowView(y=valve.m_flow) annotation(Placement(transformation(origin={67,1},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput massFlow annotation(Placement(transformation(origin={138,1},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,26},extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Interfaces.RealOutput pressure annotation(Placement(transformation(origin={153,-77},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,-26},extent={{-8,-8},{8,8}})));
equation
  connect(upstream.ports[1],valve.port_a) annotation(Line(points={{-148,55},{-13,55}},color={0,127,255}));
  connect(valve.port_b,downstream.ports[1]) annotation(Line(points={{21,55},{147,55}},color={0,127,255}));
  connect(valveCommand.y,valve.opening) annotation(Line(points={{17.2,121},{22.2,121},{4,121},{4,68.6}},color={0,0,127}));
  connect(upstream.ports[2],sensor.port) annotation(Line(points={{-148,55},{-109,55},{-109,-68},{-109,-63}},color={0,127,255}));
  connect(massFlowView.y,massFlow) annotation(Line(points={{80.2,1},{138,1}},color={0,0,127}));
  connect(sensor.p,pressure) annotation(Line(points={{-90.3,-46},{153,-46},{153,-77}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Polygon(points={{-49.4,39.8},{-49.4,-5.8},{49.4,39.8},{49.4,-5.8},{-49.4,39.8}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.8),
        Line(points={{0,17},{0,61.84}}, color={64,64,64}, thickness=0.6, pattern=LinePattern.Solid),
        Line(points={{-76,17},{-50,17}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{50,17},{76,17}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Ellipse(extent={{-20,-72},{20,-32}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6),
        Line(points={{0,-52},{12,-39}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{0,-32},{0,0}}, color={0,127,255}, thickness=0.6, pattern=LinePattern.Solid),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> Compare isenthalpic two-port paths and a zero-flow pressure reading</p><p><b>Interfaces</b> Native-constrained replaceable devices, host input and measurement outputs.</p><p><b>Parameters</b> The host sets nominal parameters in the original component constraint.</p><p><b>Behavior</b> Only the device implementations differ between the example branches.</p><p><b>Initialization</b> See nominal and modified values in the graphical view.</p><p><b>Activation and recovery</b> Set by each device instance.</p><p><b>Limitations</b> A compact fixture for the declared comparison.</p><p><b>Observables</b> massFlow; pressure</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p></html>"));
end ValveCircuit;
