within MSLFaults.Fluid;
package Pipes "Pipes with additional pressure-loss relations"
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-66.3,-24.65},{66.3,24.65}}, lineColor={255,0,0}, fillColor={0,127,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.75, radius=0),
        Line(points={{-22.95,-22.1},{-22.95,22.1}}, color={255,255,255}, thickness=0.6, pattern=LinePattern.Dash),
        Line(points={{22.95,-22.1},{22.95,22.1}}, color={255,255,255}, thickness=0.6, pattern=LinePattern.Dash),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }), Documentation(info="<html><p>Static and dynamic pipes with a series quadratic pressure-loss element. Faults retain the fluid connection topology.</p></html>"));
end Pipes;
