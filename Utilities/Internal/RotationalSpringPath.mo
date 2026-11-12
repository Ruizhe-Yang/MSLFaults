within MSLFaults.Utilities.Internal;
model RotationalSpringPath "Prescribe a finite angular deflection of a grounded spring"
  Modelica.Blocks.Sources.RealExpression angleCommand(y=0.02*time) annotation(Placement(transformation(origin={-181,48},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Mechanics.Rotational.Sources.Position drive(exact=true) annotation(Placement(transformation(origin={-97,48},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Mechanics.Rotational.Components.Spring spring annotation(Placement(transformation(origin={2,48},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Mechanics.Rotational.Components.Spring(c=10);
  Modelica.Mechanics.Rotational.Components.Fixed ground annotation(Placement(transformation(origin={96,48},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Sources.RealExpression torqueView(y=spring.tau) annotation(Placement(transformation(origin={63,-23},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput torque annotation(Placement(transformation(origin={134,-23},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
equation
  connect(angleCommand.y,drive.phi_ref) annotation(Line(points={{-167.8,48},{-111.4,48}},color={0,0,127}));
  connect(drive.flange,spring.flange_a) annotation(Line(points={{-85,48},{-13,48}},color={0,0,0}));
  connect(spring.flange_b,ground.flange) annotation(Line(points={{17,48},{96,48}},color={0,0,0}));
  connect(torqueView.y,torque) annotation(Line(points={{76.2,-23},{134,-23}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Line(points={{-61.6,12},{-42.4,12},{-36.8,29.6},{-25.6,-5.6},{-14.4,29.6},{-3.2,-5.6},{8,29.6},{19.2,-5.6},{30.4,29.6},{41.6,12},{61.6,12}}, color={255,0,0}, thickness=0.85, pattern=LinePattern.Solid),
        Line(points={{-47,52},{-36,66},{-13,72},{10,68},{27,56}}, color={64,64,64}, thickness=0.7, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-39,-43},{39,-72}}, textString="tau", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> Prescribe a finite angular deflection of a grounded spring</p><p><b>Interfaces</b> Native-constrained replaceable devices, host input and measurement outputs.</p><p><b>Parameters</b> The host sets nominal parameters in the original component constraint.</p><p><b>Behavior</b> Only the device implementations differ between the example branches.</p><p><b>Initialization</b> See nominal and modified values in the graphical view.</p><p><b>Activation and recovery</b> Set by each device instance.</p><p><b>Limitations</b> A compact fixture for the declared comparison.</p><p><b>Observables</b> torque</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p></html>"));
end RotationalSpringPath;
