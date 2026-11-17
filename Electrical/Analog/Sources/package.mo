within MSLFaults.Electrical.Analog;
package Sources "Prescribed electrical sources with output faults"
  extends Modelica.Icons.Package;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Ellipse(extent={{-51,-51},{51,51}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8),
        Line(points={{-23.8,0},{23.8,0}}, color={0,0,255}, thickness=0.9, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
  annotation(Documentation(info="<html><p>Constant, step, ramp, pulse, sinusoidal, signal-controlled and dual-rail sources. Configure the nominal waveform separately from the fault activation window.</p></html>"));
end Sources;
