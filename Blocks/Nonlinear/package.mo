within MSLFaults.Blocks;
package Nonlinear "Delays and signal limits"
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-56.1,-32.3},{-24.65,-32.3},{24.65,32.3},{56.1,32.3}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{-56.1,0},{56.1,0}}, color={128,128,128}, thickness=0.4, pattern=LinePattern.Solid),
        Line(points={{0,-48.45},{0,48.45}}, color={128,128,128}, thickness=0.4, pattern=LinePattern.Solid),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
  annotation(Documentation(info="<html><p>Causal delay and limiting blocks with configurable targets and recovery.</p></html>"));
end Nonlinear;
