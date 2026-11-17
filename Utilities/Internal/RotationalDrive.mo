within MSLFaults.Utilities.Internal;
model RotationalDrive "Torque-driven inertia with a grounded damper"
  Modelica.Blocks.Sources.Constant constantTorque(k=1) annotation(Placement(transformation(origin={-177,41},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Mechanics.Rotational.Sources.Torque drive annotation(Placement(transformation(origin={-104,41},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Mechanics.Rotational.Components.Inertia rotor annotation(Placement(transformation(origin={-22,41},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Mechanics.Rotational.Components.Inertia(J=0.1,w(start=0,fixed=true),phi(start=0,fixed=true));
  replaceable Modelica.Mechanics.Rotational.Components.Damper damper annotation(Placement(transformation(origin={65,41},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Mechanics.Rotational.Components.Damper(d=0.1);
  Modelica.Mechanics.Rotational.Components.Fixed ground annotation(Placement(transformation(origin={143,41},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Sources.RealExpression speedView(y=rotor.w) annotation(Placement(transformation(origin={61,-26},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput speed annotation(Placement(transformation(origin={132,-26},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
equation
  connect(constantTorque.y,drive.tau) annotation(Line(points={{-163.8,41},{-118.4,41}},color={0,0,127}));
  connect(drive.flange,rotor.flange_a) annotation(Line(points={{-92,41},{-37,41}},color={0,0,0}));
  connect(rotor.flange_b,damper.flange_a) annotation(Line(points={{-7,41},{50,41}},color={0,0,0}));
  connect(damper.flange_b,ground.flange) annotation(Line(points={{80,41},{143,41}},color={0,0,0}));
  connect(speedView.y,speed) annotation(Line(points={{74.2,-26},{132,-26}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Rectangle(extent={{-68,-15},{-9,51}}, lineColor={64,64,64}, fillColor={235,235,235}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6, radius=0),
        Line(points={{-83,18},{-68,18}}, color={64,64,64}, thickness=2, pattern=LinePattern.Solid),
        Line(points={{-9,18},{9,18}}, color={64,64,64}, thickness=2, pattern=LinePattern.Solid),
        Line(points={{9.08,18},{23.36,18},{23.36,5.82},{52.34,5.82}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{23.36,18},{23.36,30.18},{52.34,30.18}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{44.78,7.92},{44.78,28.08}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{44.78,18},{72.92,18}}, color={64,64,64}, thickness=0.8, pattern=LinePattern.Solid),
        Text(extent={{-56,4},{-21,35}}, textString="J", textColor={64,64,64}, fontSize=0),
        Text(extent={{24,-36},{59,-64}}, textString="d", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p>Apply a constant torque of 1 N.m to a replaceable inertia connected to a grounded viscous damper. Nominal constraints use 0.1 kg.m2, zero initial angle and speed, and 0.1 N.m.s/rad damping. <code>speed</code> reports rotor angular velocity.</p><p>Redeclare rotor or damper to separate changes in transient response and steady speed.</p></html>"));
end RotationalDrive;
