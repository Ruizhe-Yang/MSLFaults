within MSLFaults.Utilities.Internal;
model ElectromagneticCircuit "Electrical excitation of a replaceable magnetic converter"
  Modelica.Electrical.Analog.Sources.StepVoltage source(V=12,startTime=0.1) annotation(Placement(transformation(origin={-235,6},extent={{-17,-17},{17,17}},rotation=270)));
  Modelica.Electrical.Analog.Basic.Resistor series(R=10) annotation(Placement(transformation(origin={-151,98},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Electrical.Analog.Basic.Ground electricalGround annotation(Placement(transformation(origin={-235,-81},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverter converter annotation(Placement(transformation(origin={-39,34},extent={{-22,-22},{22,22}},rotation=0))) constrainedby Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverter(N=100,i(start=0,fixed=true));
  Modelica.Magnetic.FluxTubes.Basic.ConstantReluctance load(R_m=1000) annotation(Placement(transformation(origin={107,86},extent={{-17,-17},{17,17}},rotation=0)));
  Modelica.Magnetic.FluxTubes.Basic.Ground magneticGround annotation(Placement(transformation(origin={108,-27},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Sources.RealExpression currentView(y=converter.i) annotation(Placement(transformation(origin={173,-94},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput current annotation(Placement(transformation(origin={244,-94},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,28},extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Sources.RealExpression fluxView(y=load.Phi) annotation(Placement(transformation(origin={172,29},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput flux annotation(Placement(transformation(origin={243,29},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,-28},extent={{-8,-8},{8,8}})));
equation
  connect(source.p,series.p) annotation(Line(points={{-235,23},{-235,98},{-163,98}},color={0,0,255}));
  connect(series.n,converter.p) annotation(Line(points={{-139,98},{-100,98},{-100,56},{-61,56}},color={0,0,255}));
  connect(source.n,electricalGround.p) annotation(Line(points={{-235,-11},{-235,-69}},color={0,0,255}));
  connect(converter.n,electricalGround.p) annotation(Line(points={{-61,12},{-151,12},{-151,-63},{-235,-63},{-235,-69}},color={0,0,255}));
  connect(converter.port_p,load.port_p) annotation(Line(points={{-17,56},{36.5,56},{36.5,86},{90,86}},color={255,127,0}));
  connect(load.port_n,magneticGround.port) annotation(Line(points={{124,86},{129,86},{108,86},{108,-15}},color={255,127,0}));
  connect(converter.port_n,magneticGround.port) annotation(Line(points={{-17,12},{108,12},{108,-15}},color={255,127,0}));
  connect(currentView.y,current) annotation(Line(points={{186.2,-94},{244,-94}},color={0,0,127}));
  connect(fluxView.y,flux) annotation(Line(points={{185.2,29},{243,29}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Rectangle(extent={{-39.9,-15.9},{39.9,49.9}}, lineColor={255,127,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=1.3, radius=0),
        Rectangle(extent={{-23.8,-0.5},{23.8,34.5}}, lineColor={255,127,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8, radius=0),
        Line(points={{-43.4,-8.9},{-40.138,-7.822},{-37.1,-6.744},{-34.489,-5.666},{-32.487,-4.581},{-31.227,-3.503},{-30.8,-2.425},{-31.227,-1.347},{-32.487,-0.269},{-34.489,0.816},{-37.1,1.894},{-40.138,2.972},{-43.4,4.05},{-43.4,4.05},{-40.138,5.128},{-37.1,6.206},{-34.489,7.284},{-32.487,8.369},{-31.227,9.447},{-30.8,10.525},{-31.227,11.603},{-32.487,12.681},{-34.489,13.766},{-37.1,14.844},{-40.138,15.922},{-43.4,17},{-43.4,17},{-40.138,18.078},{-37.1,19.156},{-34.489,20.234},{-32.487,21.319},{-31.227,22.397},{-30.8,23.475},{-31.227,24.553},{-32.487,25.631},{-34.489,26.716},{-37.1,27.794},{-40.138,28.872},{-43.4,29.95},{-43.4,29.95},{-40.138,31.028},{-37.1,32.106},{-34.489,33.184},{-32.487,34.269},{-31.227,35.347},{-30.8,36.425},{-31.227,37.503},{-32.487,38.581},{-34.489,39.666},{-37.1,40.744},{-40.138,41.822},{-43.4,42.9}}, color={255,0,0}, thickness=0.85, pattern=LinePattern.Solid),
        Line(points={{3.5,58.3},{25.9,58.3}}, color={255,127,0}, thickness=0.7, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Ellipse(extent={{29,-70},{69,-30}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6),
        Line(points={{49,-50},{61,-37}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Text(extent={{-61,-41},{13,-70}}, textString="I -> Phi", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p>A 12 V step and 10 Ω series resistor drive an electromagnetic converter connected to a reluctance of 1000 A/Wb. The converter constraint uses 100 turns and a fixed zero initial current. Separate electrical and magnetic references complete the networks.</p><p>Redeclare converter and inspect current and flux together.</p></html>"));
end ElectromagneticCircuit;
