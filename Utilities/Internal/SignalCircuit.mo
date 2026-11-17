within MSLFaults.Utilities.Internal;
model SignalCircuit "Common ramp for gain, first-order and limiter comparisons"
  Modelica.Blocks.Sources.RealExpression signal(y=time) annotation(Placement(transformation(origin={-276,27},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Blocks.Math.Gain gain annotation(Placement(transformation(origin={-166,27},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Blocks.Math.Gain(k=2);
  replaceable Modelica.Blocks.Continuous.FirstOrder lag annotation(Placement(transformation(origin={-49,27},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Blocks.Continuous.FirstOrder(k=1,T=0.5,y_start=0,initType=Modelica.Blocks.Types.Init.InitialState);
  replaceable Modelica.Blocks.Nonlinear.Limiter limit annotation(Placement(transformation(origin={67,27},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Blocks.Nonlinear.Limiter(uMax=3,uMin=-3);
  Modelica.Blocks.Interfaces.RealOutput gainValue annotation(Placement(transformation(origin={245,110},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,45},extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Interfaces.RealOutput lagValue annotation(Placement(transformation(origin={245,27},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Interfaces.RealOutput limitValue annotation(Placement(transformation(origin={245,-63},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,-45},extent={{-8,-8},{8,8}})));
equation
  connect(signal.y,gain.u) annotation(Line(points={{-262.8,27},{-184,27}},color={0,0,127}));
  connect(signal.y,lag.u) annotation(Line(points={{-262.8,27},{-257.8,27},{-257.8,9},{-72,9},{-72,27},{-67,27}},color={0,0,127}));
  connect(signal.y,limit.u) annotation(Line(points={{-262.8,27},{-257.8,27},{-257.8,9},{44,9},{44,27},{49,27}},color={0,0,127}));
  connect(gain.y,gainValue) annotation(Line(points={{-149.5,27},{-143.5,27},{-143.5,110},{245,110}},color={0,0,127}));
  connect(lag.y,lagValue) annotation(Line(points={{-32.5,27},{-27.5,27},{-27.5,9},{245,9},{245,27}},color={0,0,127}));
  connect(limit.y,limitValue) annotation(Line(points={{83.5,27},{167.25,27},{167.25,-63},{245,-63}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Line(points={{-42.21,49.51},{-42.21,-15.48},{44.22,-15.48}}, color={128,128,128}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-37.52,-7.44},{-22.78,-7.44},{-7.37,23.38},{11.39,38.79},{39.53,42.14}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{-54,-50},{-25,-50},{-25,-24},{24,-24},{24,-50},{55,-50}}, color={64,64,64}, thickness=0.65, pattern=LinePattern.Solid),
        Text(extent={{-47,60},{47,82}}, textString="u -> y", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p>Feed <code>time</code> independently to a gain, a first-order block and a limiter. Native constraints use gain 2, time constant 0.5 s with a fixed zero initial state, and limits ±3. Outputs expose the three separate responses.</p><p>Redeclare gain, lag or limit and inspect gainValue, lagValue or limitValue.</p></html>"));
end SignalCircuit;
