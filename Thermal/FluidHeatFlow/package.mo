within MSLFaults.Thermal;
package FluidHeatFlow "Faults in simplified thermal-fluid networks"
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-62.05,-25.5},{62.05,25.5}}, lineColor={255,0,0}, fillColor={0,127,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.7, radius=0),
        Line(points={{-35.7,0},{35.7,0}}, color={255,255,255}, thickness=0.8, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Line(points={{-29.75,-40.8},{-29.75,-59.5}}, color={191,0,0}, thickness=0.7, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Line(points={{0,-40.8},{0,-59.5}}, color={191,0,0}, thickness=0.7, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Line(points={{29.75,-40.8},{29.75,-59.5}}, color={191,0,0}, thickness=0.7, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }), Documentation(info="<html><p>Components and sensors for <code>Modelica.Thermal.FluidHeatFlow</code>, using its <code>FlowPort</code> connectors and medium record. These are distinct from <code>Modelica.Fluid</code> interfaces.</p></html>"));
end FluidHeatFlow;
