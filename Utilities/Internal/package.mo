within MSLFaults.Utilities;
package Internal "Internal activation and example assemblies"
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-40.8,-40.8},{32.3,32.3}}, color={64,64,64}, thickness=5, pattern=LinePattern.Solid),
        Polygon(points={{13.6,31.45},{20.4,50.15},{40.8,55.25},{29.75,39.1},{42.5,26.35},{56.95,38.25},{53.55,17},{35.7,10.2},{13.6,31.45}}, lineColor={64,64,64}, fillColor={235,235,235}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6),
        Ellipse(extent={{-51.85,-52.7},{-29.75,-30.6}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.7),
        Line(points={{-64.6,46.75},{-64.6,63.75},{-46.75,63.75}}, color={128,128,128}, thickness=0.7, pattern=LinePattern.Solid),
        Line(points={{46.75,-63.75},{64.6,-63.75},{64.6,-46.75}}, color={128,128,128}, thickness=0.7, pattern=LinePattern.Solid),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
  annotation(Documentation(info="<html><p>The activation helper and complete circuits used to compare native, normal and faulted components. These assemblies support the examples; applications may connect the public components directly.</p></html>"));
end Internal;
