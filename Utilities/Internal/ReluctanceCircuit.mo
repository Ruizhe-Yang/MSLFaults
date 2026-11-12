within MSLFaults.Utilities.Internal;
model ReluctanceCircuit "Provide a magnetic potential difference and a magnetic reference"
  Modelica.Magnetic.FluxTubes.Sources.ConstantMagneticPotentialDifference mmf(V_m=100) annotation(Placement(transformation(origin={-123,17},extent={{-16,-16},{16,16}},rotation=270)));
  Modelica.Magnetic.FluxTubes.Basic.Ground ground annotation(Placement(transformation(origin={-123,-70},extent={{-12,-12},{12,12}},rotation=0)));
  replaceable Modelica.Magnetic.FluxTubes.Basic.ConstantReluctance device annotation(Placement(transformation(origin={10,76},extent={{-15,-15},{15,15}},rotation=0))) constrainedby Modelica.Magnetic.FluxTubes.Basic.ConstantReluctance(R_m=1000);
  Modelica.Blocks.Sources.RealExpression fluxView(y=device.Phi) annotation(Placement(transformation(origin={64,-10},extent={{-12,-12},{12,12}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput flux annotation(Placement(transformation(origin={135,-10},extent={{-7,-7},{7,7}},rotation=0),iconTransformation(origin={100,0},extent={{-8,-8},{8,8}})));
equation
  connect(mmf.port_p,device.port_p) annotation(Line(points={{-123,33},{-123,76},{-5,76}},color={255,127,0}));
  connect(device.port_n,ground.port) annotation(Line(points={{25,76},{30,76},{30,-53},{-123,-53},{-123,-58}},color={255,127,0}));
  connect(mmf.port_n,ground.port) annotation(Line(points={{-123,1},{-123,-58}},color={255,127,0}));
  connect(fluxView.y,flux) annotation(Line(points={{77.2,-10},{135,-10}},color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true)),
    Icon(graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Rectangle(extent={{-39.9,-15.9},{39.9,49.9}}, lineColor={255,127,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=1.3, radius=0),
        Rectangle(extent={{-23.8,-0.5},{23.8,34.5}}, lineColor={255,127,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8, radius=0),
        Line(points={{-43.4,-8.9},{-40.138,-7.822},{-37.1,-6.744},{-34.489,-5.666},{-32.487,-4.581},{-31.227,-3.503},{-30.8,-2.425},{-31.227,-1.347},{-32.487,-0.269},{-34.489,0.816},{-37.1,1.894},{-40.138,2.972},{-43.4,4.05},{-43.4,4.05},{-40.138,5.128},{-37.1,6.206},{-34.489,7.284},{-32.487,8.369},{-31.227,9.447},{-30.8,10.525},{-31.227,11.603},{-32.487,12.681},{-34.489,13.766},{-37.1,14.844},{-40.138,15.922},{-43.4,17},{-43.4,17},{-40.138,18.078},{-37.1,19.156},{-34.489,20.234},{-32.487,21.319},{-31.227,22.397},{-30.8,23.475},{-31.227,24.553},{-32.487,25.631},{-34.489,26.716},{-37.1,27.794},{-40.138,28.872},{-43.4,29.95},{-43.4,29.95},{-40.138,31.028},{-37.1,32.106},{-34.489,33.184},{-32.487,34.269},{-31.227,35.347},{-30.8,36.425},{-31.227,37.503},{-32.487,38.581},{-34.489,39.666},{-37.1,40.744},{-40.138,41.822},{-43.4,42.9}}, color={255,0,0}, thickness=0.85, pattern=LinePattern.Solid),
        Line(points={{3.5,58.3},{25.9,58.3}}, color={255,127,0}, thickness=0.7, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Ellipse(extent={{29,-70},{69,-30}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6),
        Line(points={{49,-50},{61,-37}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Text(extent={{-50,-43},{10,-69}}, textString="Rm", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> Provide a magnetic potential difference and a magnetic reference</p><p><b>Interfaces</b> Native-constrained replaceable devices, host input and measurement outputs.</p><p><b>Parameters</b> The host sets nominal parameters in the original component constraint.</p><p><b>Behavior</b> Only the device implementations differ between the example branches.</p><p><b>Initialization</b> See nominal and modified values in the graphical view.</p><p><b>Activation and recovery</b> Set by each device instance.</p><p><b>Limitations</b> A compact fixture for the declared comparison.</p><p><b>Observables</b> flux</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.MagneticReluctance\">MSLFaults.Examples.MagneticReluctance</a></p></html>"));
end ReluctanceCircuit;
