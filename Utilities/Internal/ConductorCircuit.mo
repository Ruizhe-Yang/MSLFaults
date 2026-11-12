within MSLFaults.Utilities.Internal;
model ConductorCircuit "Compare branches with a common electrical excitation"
  Modelica.Electrical.Analog.Sources.ConstantVoltage voltage(V=12) annotation(Placement(transformation(origin={-125,12},extent={{-12,-12},{12,12}},rotation=270)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(transformation(origin={-125,-66},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Electrical.Analog.Basic.Conductor device annotation(Placement(transformation(origin={17,76},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Electrical.Analog.Basic.Conductor(G=0.1);
  Modelica.Blocks.Sources.RealExpression readoutView(y=device.i) annotation(Placement(transformation(origin={81,19},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput readout annotation(Placement(transformation(origin={152,19},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
equation
  connect(voltage.n,ground.p) annotation(Line(points={{-125,0},{-125,-54}},color={0,0,255}));
  connect(voltage.p,device.p) annotation(Line(points={{-125,24},{-125,76},{2,76}},color={0,0,255}));
  connect(device.n,ground.p) annotation(Line(points={{32,76},{37,76},{37,-49},{-125,-49},{-125,-54}},color={0,0,255}));
  connect(readoutView.y,readout) annotation(Line(points={{94.2,19},{152,19}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Line(points={{-56.25,28},{-30,28}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Rectangle(extent={{-30,15.25},{30,40.75}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.75, radius=0),
        Line(points={{30,28},{56.25,28}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Text(extent={{-22.5,18.25},{22.5,37.75}}, textString="G", textColor={64,64,64}, fontSize=0),
        Line(points={{-56,28},{-66,28},{-66,-42},{28,-42}}, color={0,0,255}, thickness=0.6, pattern=LinePattern.Solid),
        Ellipse(extent={{28,-62},{68,-22}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6),
        Line(points={{48,-42},{60,-29}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{68,-42},{74,-42},{74,28},{56,28}}, color={0,0,255}, thickness=0.6, pattern=LinePattern.Solid),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> Compare branches with a common electrical excitation</p><p><b>Interfaces</b> Native-constrained replaceable devices, host input and measurement outputs.</p><p><b>Parameters</b> The host sets nominal parameters in the original component constraint.</p><p><b>Behavior</b> Only the device implementations differ between the example branches.</p><p><b>Initialization</b> See nominal and modified values in the graphical view.</p><p><b>Activation and recovery</b> Set by each device instance.</p><p><b>Limitations</b> A compact fixture for the declared comparison.</p><p><b>Observables</b> readout</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p></html>"));
end ConductorCircuit;
