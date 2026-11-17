within MSLFaults.Electrical.Analog;
package Semiconductors "Semiconductor devices with configurable degradation"
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-62.9,0},{-24.65,0}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Polygon(points={{-24.65,-31.45},{-24.65,31.45},{23.8,0},{-24.65,-31.45}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.8),
        Line(points={{26.35,-31.45},{26.35,31.45}}, color={255,0,0}, thickness=0.85, pattern=LinePattern.Solid),
        Line(points={{26.35,0},{62.9,0}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
  annotation(Documentation(info="<html><p>PN and Zener diodes, MOS and bipolar transistors, and an NPN circuit stage. Faults modify junction, transport, channel or leakage relations.</p></html>"));
end Semiconductors;
