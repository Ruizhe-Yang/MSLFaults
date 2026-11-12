within MSLFaults.Utilities.Internal;
model ConvectiveCircuit "Heat a lumped capacity through the declared path to ambient"
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow heater(Q_flow=100) annotation(Placement(transformation(origin={-153,62},extent={{-15,-15},{15,15}},rotation=0)));
  replaceable Modelica.Thermal.HeatTransfer.Components.HeatCapacitor capacity annotation(Placement(transformation(origin={-42,-24},extent={{-17,-17},{17,17}},rotation=0))) constrainedby Modelica.Thermal.HeatTransfer.Components.HeatCapacitor(C=100,T(start=300,fixed=true));
  replaceable Modelica.Thermal.HeatTransfer.Components.Convection heatPath annotation(Placement(transformation(origin={14,62},extent={{-17,-17},{17,17}},rotation=0))) constrainedby Modelica.Thermal.HeatTransfer.Components.Convection;
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature ambient(T=300) annotation(Placement(transformation(origin={141,62},extent={{-12,-12},{12,12}},rotation=180)));
  Modelica.Blocks.Sources.Constant gc(k=10) annotation(Placement(transformation(origin={13,120},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Sources.RealExpression temperatureView(y=capacity.T) annotation(Placement(transformation(origin={69,-44},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput temperature annotation(Placement(transformation(origin={140,-44},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
equation
  connect(heater.port,heatPath.solid) annotation(Line(points={{-138,62},{-3,62}},color={191,0,0}));
  connect(capacity.port,heatPath.solid) annotation(Line(points={{-42,-41},{-42,-46},{-42,62},{-3,62}},color={191,0,0}));
  connect(heatPath.fluid,ambient.port) annotation(Line(points={{31,62},{129,62}},color={191,0,0}));
  connect(gc.y,heatPath.Gc) annotation(Line(points={{26.2,120},{31.2,120},{14,120},{14,79}},color={0,0,127}));
  connect(temperatureView.y,temperature) annotation(Line(points={{82.2,-44},{140,-44}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Rectangle(extent={{-43.4,-16.7},{-27.3,54.7}}, lineColor={128,128,128}, fillColor={235,235,235}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6, radius=0),
        Rectangle(extent={{27.3,-16.7},{43.4,54.7}}, lineColor={128,128,128}, fillColor={235,235,235}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6, radius=0),
        Line(points={{-21.7,31.6},{21.7,31.6}}, color={191,0,0}, thickness=0.9, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Line(points={{21.7,6.4},{-21.7,6.4}}, color={191,0,0}, thickness=0.9, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Ellipse(extent={{-59.75,-64.1},{-46.25,-50.6}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.65),
        Rectangle(extent={{-56.6,-51.5},{-49.4,-13.7}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.65, radius=2.25),
        Line(points={{-53,-57.8},{-53,-20}}, color={191,0,0}, thickness=2, pattern=LinePattern.Solid),
        Line(points={{-46.25,-39.8},{-39.05,-39.8}}, color={128,128,128}, thickness=0.55, pattern=LinePattern.Solid),
        Line(points={{-46.25,-28.55},{-39.05,-28.55}}, color={128,128,128}, thickness=0.55, pattern=LinePattern.Solid),
        Line(points={{-46.25,-17.3},{-39.05,-17.3}}, color={128,128,128}, thickness=0.55, pattern=LinePattern.Solid),
        Line(points={{-53,-13},{-53,19},{-44,19}}, color={191,0,0}, thickness=0.6, pattern=LinePattern.Solid),
        Line(points={{44,19},{65,19},{65,-41}}, color={191,0,0}, thickness=0.6, pattern=LinePattern.Solid),
        Text(extent={{-13,-31},{38,-61}}, textString="Gc", textColor={64,64,64}, fontSize=0),
        Line(points={{13,75},{13,56}}, color={0,0,255}, thickness=0.6, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> Heat a lumped capacity through the declared path to ambient</p><p><b>Interfaces</b> Native-constrained replaceable devices, host input and measurement outputs.</p><p><b>Parameters</b> The host sets nominal parameters in the original component constraint.</p><p><b>Behavior</b> Only the device implementations differ between the example branches.</p><p><b>Initialization</b> See nominal and modified values in the graphical view.</p><p><b>Activation and recovery</b> Set by each device instance.</p><p><b>Limitations</b> A compact fixture for the declared comparison.</p><p><b>Observables</b> temperature</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p></html>"));
end ConvectiveCircuit;
