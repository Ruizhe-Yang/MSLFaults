within MSLFaults.Utilities.Internal;
model TranslationalDrive "Force-driven mass with parallel spring and damper"
  Modelica.Blocks.Sources.Constant constantForce(k=1) annotation(Placement(transformation(origin={-180,66},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Mechanics.Translational.Sources.Force drive annotation(Placement(transformation(origin={-99,66},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Mechanics.Translational.Components.Mass mass annotation(Placement(transformation(origin={-15,66},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Mechanics.Translational.Components.Mass(m=1,s(start=0,fixed=true),v(start=0,fixed=true));
  replaceable Modelica.Mechanics.Translational.Components.Spring spring annotation(Placement(transformation(origin={68,66},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Mechanics.Translational.Components.Spring(c=100);
  replaceable Modelica.Mechanics.Translational.Components.Damper damper annotation(Placement(transformation(origin={68,-19},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Mechanics.Translational.Components.Damper(d=2);
  Modelica.Mechanics.Translational.Components.Fixed ground annotation(Placement(transformation(origin={159,32},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Sources.RealExpression positionView(y=mass.s) annotation(Placement(transformation(origin={61,-91},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput position annotation(Placement(transformation(origin={132,-91},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
equation
  connect(constantForce.y,drive.f) annotation(Line(points={{-166.8,66},{-113.4,66}},color={0,0,127}));
  connect(drive.flange,mass.flange_a) annotation(Line(points={{-87,66},{-30,66}},color={0,127,0}));
  connect(mass.flange_b,spring.flange_a) annotation(Line(points={{0,66},{53,66}},color={0,127,0}));
  connect(mass.flange_b,damper.flange_a) annotation(Line(points={{0,66},{26.5,66},{26.5,-19},{53,-19}},color={0,127,0}));
  connect(spring.flange_b,ground.flange) annotation(Line(points={{83,66},{159,66},{159,32}},color={0,127,0}));
  connect(damper.flange_b,ground.flange) annotation(Line(points={{83,-19},{159,-19},{159,32}},color={0,127,0}));
  connect(positionView.y,position) annotation(Line(points={{74.2,-91},{132,-91}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Rectangle(extent={{-71,-24},{-25,36}}, lineColor={64,64,64}, fillColor={235,235,235}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.65, radius=0),
        Text(extent={{-63,-13},{-32,24}}, textString="m", textColor={64,64,64}, fontSize=0),
        Line(points={{-10.19,24},{1.09,24},{4.38,34.34},{10.96,13.66},{17.54,34.34},{24.12,13.66},{30.7,34.34},{37.28,13.66},{43.86,34.34},{50.44,24},{62.19,24}}, color={255,0,0}, thickness=0.85, pattern=LinePattern.Solid),
        Line(points={{-9.72,-33},{6.26,-33},{6.26,-46.63},{38.69,-46.63}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{6.26,-33},{6.26,-19.37},{38.69,-19.37}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{30.23,-44.28},{30.23,-21.72}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{30.23,-33},{61.72,-33}}, color={64,64,64}, thickness=0.8, pattern=LinePattern.Solid),
        Line(points={{-25,7},{-13,7},{-13,-33},{-10,-33}}, color={64,64,64}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{62,24},{69,24},{69,-33},{62,-33}}, color={64,64,64}, thickness=0.65, pattern=LinePattern.Solid),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p>Apply 1 N to a replaceable mass supported by a parallel spring and damper. Native constraints use 1 kg, 100 N/m and 2 N.s/m, with fixed zero initial position and velocity. <code>position</code> reports the mass position.</p><p>Redeclare one of mass, spring or damper at a time and compare position.</p></html>"));
end TranslationalDrive;
