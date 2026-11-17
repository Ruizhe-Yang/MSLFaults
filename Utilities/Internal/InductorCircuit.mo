within MSLFaults.Utilities.Internal;
model InductorCircuit "RL circuit with a replaceable inductor"
  parameter Modelica.Units.SI.Time stepTime=0.01 "Start of the applied voltage step";
  Modelica.Electrical.Analog.Sources.StepVoltage voltage(V=12,startTime=stepTime) annotation(Placement(transformation(origin={-151,12},extent={{-12,-12},{12,12}},rotation=270)));
  Modelica.Electrical.Analog.Basic.Resistor series(R=10) annotation(Placement(transformation(origin={-68,76},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(transformation(origin={-151,-66},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Electrical.Analog.Basic.Inductor device annotation(Placement(transformation(origin={17,76},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Electrical.Analog.Basic.Inductor(L=0.1,i(start=0,fixed=true));
  Modelica.Blocks.Sources.RealExpression readoutView(y=device.i) annotation(Placement(transformation(origin={81,19},extent={{-12,-12},{12,12}},rotation=0)));
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
        Line(points={{-55.5,28},{-39,28}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{39,28},{55.5,28}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{-39,28},{-37.05,32.6353},{-35.1,36.8168},{-33.15,40.1353},{-31.2,42.2658},{-29.25,43},{-27.3,42.2658},{-25.35,40.1353},{-23.4,36.8168},{-21.45,32.6353},{-19.5,28}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{-19.5,28},{-17.55,32.6353},{-15.6,36.8168},{-13.65,40.1353},{-11.7,42.2658},{-9.75,43},{-7.8,42.2658},{-5.85,40.1353},{-3.9,36.8168},{-1.95,32.6353},{0,28}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{0,28},{1.95,32.6353},{3.9,36.8168},{5.85,40.1353},{7.8,42.2658},{9.75,43},{11.7,42.2658},{13.65,40.1353},{15.6,36.8168},{17.55,32.6353},{19.5,28}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{19.5,28},{21.45,32.6353},{23.4,36.8168},{25.35,40.1353},{27.3,42.2658},{29.25,43},{31.2,42.2658},{33.15,40.1353},{35.1,36.8168},{37.05,32.6353},{39,28}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{-56,28},{-66,28},{-66,-42},{28,-42}}, color={0,0,255}, thickness=0.6, pattern=LinePattern.Solid),
        Ellipse(extent={{28,-62},{68,-22}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6),
        Line(points={{48,-42},{60,-29}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{68,-42},{74,-42},{74,28},{56,28}}, color={0,0,255}, thickness=0.6, pattern=LinePattern.Solid),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p>A 12 V step drives a 10 Ω series resistor and a replaceable inductor. The constraint specifies 0.1 H and a fixed zero initial current. <code>readout</code> reports inductor current.</p><p>Redeclare device, retain the current initialisation and compare the current rise after stepTime.</p></html>"));
end InductorCircuit;
