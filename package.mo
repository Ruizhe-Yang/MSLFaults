within;
package MSLFaults "Component fault models for Modelica Standard Library interfaces"
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-57.375,0},{-30.6,0}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Rectangle(extent={{-30.6,-13.005},{30.6,13.005}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.75, radius=0),
        Line(points={{30.6,0},{57.375,0}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Text(extent={{-22.95,-9.945},{22.95,9.945}}, textString="R", textColor={64,64,64}, fontSize=0),
        Line(points={{-40.8,-33.15},{-16.15,-33.15},{-7.65,-11.9},{0.85,-33.15},{40.8,-33.15}}, color={255,0,0}, thickness=0.8, pattern=LinePattern.Solid),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    uses(Modelica(version="4.0.0")), version="1.0.0", Documentation(info="<html><p>MSLFaults provides deterministic fault mechanisms for signal-processing and physical components associated with Modelica Standard Library 4.0.0 counterparts. Configure the local fault mode and target, then inspect the component response and activation diagnostics.</p><p>Begin with <a href=\"modelica://MSLFaults.UsersGuide\">UsersGuide</a> and <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">Examples.ResistorReplacement</a>. The library covers electrical, mechanical, thermal, fluid and magnetic networks as well as causal signal blocks.</p></html>"));
end MSLFaults;
