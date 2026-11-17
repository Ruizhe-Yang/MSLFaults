within MSLFaults.Mechanics.Translational;
package Sensors "Translational sensors with output faults"
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Ellipse(extent={{-59.5,-59.5},{59.5,59.5}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.8),
        Line(points={{-40.8,19.55},{-33.15,15.3}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-21.25,40.8},{-17,33.15}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{0,46.75},{0,37.4}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{21.25,40.8},{17,33.15}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{40.8,19.55},{33.15,15.3}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{0,0},{25.5,28.05}}, color={64,64,64}, thickness=0.9, pattern=LinePattern.Solid),
        Ellipse(extent={{-3.4,-3.4},{3.4,3.4}}, lineColor={64,64,64}, fillColor={64,64,64}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.4),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
  annotation(Documentation(info="<html><p>Absolute and relative motion, force and power measurements. Relative-motion sensors observe the difference from flange_a to flange_b; Saturation acts directly whenever selected.</p></html>"));
end Sensors;
