within MSLFaults.Utilities.Internal;
model ElectromechanicalCircuit "Motor, winding heat and speed-feedback assembly"
  parameter Real resistance=2 "Nominal winding resistance";
  Modelica.Blocks.Sources.RealExpression voltageCommand(y=12-feedback.y) annotation(Placement(transformation(origin={-272,57},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Electrical.Analog.Sources.SignalVoltage source annotation(Placement(transformation(origin={-173,44},extent={{-17,-17},{17,17}},rotation=270)));
  Modelica.Electrical.Analog.Basic.Ground electricalGround annotation(Placement(transformation(origin={-173,-28},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Electrical.Analog.Basic.Resistor winding annotation(Placement(transformation(origin={-65,116},extent={{-17,-17},{17,17}},rotation=0))) constrainedby Modelica.Electrical.Analog.Basic.Resistor(R=resistance,useHeatPort=true);
  Modelica.Electrical.Analog.Basic.RotationalEMF motor(k=0.2) annotation(Placement(transformation(origin={66,116},extent={{-19,-19},{19,19}},rotation=0)));
  Modelica.Mechanics.Rotational.Components.Inertia rotor(J=0.1,w(start=0,fixed=true),phi(start=0,fixed=true)) annotation(Placement(transformation(origin={160,116},extent={{-17,-17},{17,17}},rotation=0)));
  Modelica.Mechanics.Rotational.Components.Damper friction(d=0.1) annotation(Placement(transformation(origin={237,116},extent={{-14,-14},{14,14}},rotation=0)));
  Modelica.Mechanics.Rotational.Components.Fixed reference annotation(Placement(transformation(origin={302,116},extent={{-10,-10},{10,10}},rotation=0)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(Placement(transformation(origin={157,41},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Blocks.Math.Gain feedback annotation(Placement(transformation(origin={-65,0},extent={{-15,-15},{15,15}},rotation=180))) constrainedby Modelica.Blocks.Math.Gain(k=0.1);
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor windingHeat(C=100,T(start=300,fixed=true)) annotation(Placement(transformation(origin={-68,-104},extent={{-15,-15},{15,15}},rotation=0)));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor cooling(G=1) annotation(Placement(transformation(origin={50,-104},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature ambient(T=300) annotation(Placement(transformation(origin={148,-104},extent={{-12,-12},{12,12}},rotation=180)));
  Modelica.Blocks.Sources.RealExpression currentView(y=winding.i) annotation(Placement(transformation(origin={224,-22},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput current annotation(Placement(transformation(origin={295,-22},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,45},extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Interfaces.RealOutput speed annotation(Placement(transformation(origin={295,47},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Sources.RealExpression temperatureView(y=windingHeat.T) annotation(Placement(transformation(origin={224,-96},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput temperature annotation(Placement(transformation(origin={295,-96},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,-45},extent={{-8,-8},{8,8}})));
equation
  connect(voltageCommand.y,source.v) annotation(Line(points={{-258.8,57},{-147.6,57},{-147.6,44},{-152.6,44}},color={0,0,127}));
  connect(source.p,winding.p) annotation(Line(points={{-173,61},{-173,116},{-82,116}},color={0,0,255}));
  connect(source.n,electricalGround.p) annotation(Line(points={{-173,27},{-173,-16}},color={0,0,255}));
  connect(winding.n,motor.p) annotation(Line(points={{-48,116},{66,116},{66,140},{66,135}},color={0,0,255}));
  connect(motor.n,electricalGround.p) annotation(Line(points={{66,97},{66,24},{-173,24},{-173,-16}},color={0,0,255}));
  connect(motor.flange,rotor.flange_a) annotation(Line(points={{85,116},{143,116}},color={0,0,0}));
  connect(rotor.flange_b,friction.flange_a) annotation(Line(points={{177,116},{223,116}},color={0,0,0}));
  connect(friction.flange_b,reference.flange) annotation(Line(points={{251,116},{302,116}},color={0,0,0}));
  connect(rotor.flange_b,speedSensor.flange) annotation(Line(points={{177,116},{182,116},{182,41},{140,41},{145,41}},color={0,0,0}));
  connect(speedSensor.w,feedback.u) annotation(Line(points={{170.2,41},{175.2,41},{175.2,0},{-47,0}},color={0,0,127}));
  connect(winding.heatPort,windingHeat.port) annotation(Line(points={{-65,99},{-65,94},{-83,94},{-83,-124},{-68,-124},{-68,-119}},color={191,0,0}));
  connect(windingHeat.port,cooling.port_a) annotation(Line(points={{-68,-119},{-68,-124},{-68,-104},{38,-104}},color={191,0,0}));
  connect(cooling.port_b,ambient.port) annotation(Line(points={{62,-104},{136,-104}},color={191,0,0}));
  connect(currentView.y,current) annotation(Line(points={{237.2,-22},{295,-22}},color={0,0,127}));
  connect(speedSensor.w,speed) annotation(Line(points={{170.2,41},{295,41},{295,47}},color={0,0,127}));
  connect(temperatureView.y,temperature) annotation(Line(points={{237.2,-96},{295,-96}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Ellipse(extent={{-42,-15},{42,69}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.7),
        Text(extent={{-31,3},{31,49}}, textString="M", textColor={64,64,64}, fontSize=0),
        Line(points={{-74,29},{-42,29}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{42,29},{74,29}}, color={64,64,64}, thickness=2, pattern=LinePattern.Solid),
        Line(points={{0,-15},{0,-57}}, color={191,0,0}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{-22,-59},{22,-59}}, color={191,0,0}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{-16,-69},{16,-69}}, color={191,0,0}, thickness=0.65, pattern=LinePattern.Solid),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p>A voltage command <code>12-feedback.y</code> drives a winding resistance and rotational electromechanical converter. The rotor drives a viscous damper; the winding heat port connects to a thermal capacity and an ambient cooling path. Winding resistance and feedback gain are replaceable.</p><p>Redeclare winding or feedback; compare current, speed and temperature under the same command.</p></html>"));
end ElectromechanicalCircuit;
