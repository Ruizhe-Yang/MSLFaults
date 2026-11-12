within MSLFaults.Utilities.Internal;
model CapacitorCircuit "Compare branches with a common electrical excitation"
  parameter Modelica.Units.SI.Time stepTime=0.2;
  Modelica.Electrical.Analog.Sources.StepVoltage voltage(V=10,startTime=stepTime) annotation(Placement(transformation(origin={-151,12},extent={{-12,-12},{12,12}},rotation=270)));
  Modelica.Electrical.Analog.Basic.Resistor series(R=1000) annotation(Placement(transformation(origin={-68,76},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(transformation(origin={-151,-66},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Electrical.Analog.Basic.Capacitor device annotation(Placement(transformation(origin={17,76},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Electrical.Analog.Basic.Capacitor(C=0.001,v(start=0,fixed=true));
  Modelica.Blocks.Sources.RealExpression readoutView(y=device.v) annotation(Placement(transformation(origin={81,19},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput readout annotation(Placement(transformation(origin={152,19},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
equation
  connect(voltage.n,ground.p) annotation(Line(points={{-151,0},{-151,-54}},color={0,0,255}));
  connect(voltage.p,series.p) annotation(Line(points={{-151,24},{-151,76},{-80,76}},color={0,0,255}));
  connect(series.n,device.p) annotation(Line(points={{-56,76},{2,76}},color={0,0,255}));
  connect(device.n,ground.p) annotation(Line(points={{32,76},{37,76},{37,-49},{-151,-49},{-151,-54}},color={0,0,255}));
  connect(readoutView.y,readout) annotation(Line(points={{94.2,19},{152,19}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Line(points={{-55.5,28},{-7.5,28}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{7.5,28},{55.5,28}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{-7.5,3.25},{-7.5,52.75}}, color={255,0,0}, thickness=1, pattern=LinePattern.Solid),
        Line(points={{7.5,3.25},{7.5,52.75}}, color={255,0,0}, thickness=1, pattern=LinePattern.Solid),
        Line(points={{-56,28},{-66,28},{-66,-42},{28,-42}}, color={0,0,255}, thickness=0.6, pattern=LinePattern.Solid),
        Ellipse(extent={{28,-62},{68,-22}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6),
        Line(points={{48,-42},{60,-29}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{68,-42},{74,-42},{74,28},{56,28}}, color={0,0,255}, thickness=0.6, pattern=LinePattern.Solid),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> Compare branches with a common electrical excitation</p><p><b>Interfaces</b> Native-constrained replaceable devices, host input and measurement outputs.</p><p><b>Parameters</b> The host sets nominal parameters in the original component constraint.</p><p><b>Behavior</b> Only the device implementations differ between the example branches.</p><p><b>Initialization</b> See nominal and modified values in the graphical view.</p><p><b>Activation and recovery</b> Set by each device instance.</p><p><b>Limitations</b> A compact fixture for the declared comparison.</p><p><b>Observables</b> readout</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.CapacitorDegradation\">MSLFaults.Examples.CapacitorDegradation</a></p></html>"));
end CapacitorCircuit;
