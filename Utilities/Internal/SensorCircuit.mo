within MSLFaults.Utilities.Internal;
model SensorCircuit "Keep an identical native measuring circuit under measurement faults"
  Modelica.Blocks.Sources.RealExpression ramp(y=time) annotation(Placement(transformation(origin={-275,32},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Electrical.Analog.Sources.SignalVoltage source annotation(Placement(transformation(origin={-180,12},extent={{-17,-17},{17,17}},rotation=270)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(transformation(origin={-180,-106},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor annotation(Placement(transformation(origin={-86,110},extent={{-16,-16},{16,16}},rotation=0))) constrainedby Modelica.Electrical.Analog.Sensors.CurrentSensor;
  Modelica.Electrical.Analog.Basic.Resistor load(R=2) annotation(Placement(transformation(origin={26,110},extent={{-16,-16},{16,16}},rotation=0)));
  replaceable Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation(Placement(transformation(origin={26,-16},extent={{-16,-16},{16,16}},rotation=0))) constrainedby Modelica.Electrical.Analog.Sensors.VoltageSensor;
  Modelica.Blocks.Interfaces.RealOutput readVoltage annotation(Placement(transformation(origin={177,-46},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,38},extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Interfaces.RealOutput readCurrent annotation(Placement(transformation(origin={177,63},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Sources.RealExpression actualCurrentView(y=load.i) annotation(Placement(transformation(origin={114,-103},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput actualCurrent annotation(Placement(transformation(origin={185,-103},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,-38},extent={{-8,-8},{8,8}})));
equation
  connect(ramp.y,source.v) annotation(Line(points={{-261.8,32},{-154.6,32},{-154.6,12},{-159.6,12}},color={0,0,127}));
  connect(source.p,currentSensor.p) annotation(Line(points={{-180,29},{-180,110},{-102,110}},color={0,0,255}));
  connect(currentSensor.n,load.p) annotation(Line(points={{-70,110},{10,110}},color={0,0,255}));
  connect(load.n,ground.p) annotation(Line(points={{42,110},{47,110},{47,-89},{-180,-89},{-180,-94}},color={0,0,255}));
  connect(source.n,ground.p) annotation(Line(points={{-180,-5},{-180,-94}},color={0,0,255}));
  connect(voltageSensor.p,load.p) annotation(Line(points={{10,-16},{5,-16},{5,110},{10,110}},color={0,0,255}));
  connect(voltageSensor.n,load.n) annotation(Line(points={{42,-16},{47,-16},{47,110},{42,110}},color={0,0,255}));
  connect(voltageSensor.v,readVoltage) annotation(Line(points={{26,-33.6},{26,-46},{177,-46}},color={0,0,127}));
  connect(currentSensor.i,readCurrent) annotation(Line(points={{-86,92.4},{-86,63},{177,63}},color={0,0,127}));
  connect(actualCurrentView.y,actualCurrent) annotation(Line(points={{127.2,-103},{185,-103}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Ellipse(extent={{-70.5,-21.5},{6.5,55.5}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.8),
        Line(points={{-58.4,29.65},{-53.45,26.9}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-45.75,43.4},{-43,38.45}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-32,47.25},{-32,41.2}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-18.25,43.4},{-21,38.45}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-5.6,29.65},{-10.55,26.9}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-32,17},{-15.5,35.15}}, color={64,64,64}, thickness=0.9, pattern=LinePattern.Solid),
        Ellipse(extent={{-34.2,14.8},{-29.8,19.2}}, lineColor={64,64,64}, fillColor={64,64,64}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.4),
        Text(extent={{-61.15,7.65},{-2.85,-12.15}}, textString="V", textColor={64,64,64}, fontSize=0),
        Ellipse(extent={{14,-61},{70,-5}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.8),
        Line(points={{22.8,-23.8},{26.4,-25.8}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{32,-13.8},{34,-17.4}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{42,-11},{42,-15.4}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{52,-13.8},{50,-17.4}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{61.2,-23.8},{57.6,-25.8}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{42,-33},{54,-19.8}}, color={64,64,64}, thickness=0.9, pattern=LinePattern.Solid),
        Ellipse(extent={{40.4,-34.6},{43.6,-31.4}}, lineColor={64,64,64}, fillColor={64,64,64}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.4),
        Text(extent={{20.8,-39.8},{63.2,-54.2}}, textString="A", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> Keep an identical native measuring circuit under measurement faults</p><p><b>Interfaces</b> Native-constrained replaceable devices, host input and measurement outputs.</p><p><b>Parameters</b> The host sets nominal parameters in the original component constraint.</p><p><b>Behavior</b> Only the device implementations differ between the example branches.</p><p><b>Initialization</b> See nominal and modified values in the graphical view.</p><p><b>Activation and recovery</b> Set by each device instance.</p><p><b>Limitations</b> A compact fixture for the declared comparison.</p><p><b>Observables</b> readVoltage; readCurrent; actualCurrent</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p></html>"));
end SensorCircuit;
