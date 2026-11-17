within MSLFaults;
package Blocks "Faultable signal-processing blocks"
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-53.55,45.05},{-53.55,-37.4},{56.1,-37.4}}, color={128,128,128}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-47.6,-27.2},{-28.9,-27.2},{-9.35,11.9},{14.45,31.45},{50.15,35.7}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
  annotation(Documentation(info="<html><p>Gain, first-order dynamics, delay and limiting with explicit fault modes and activation outputs.</p></html>"));
end Blocks;
