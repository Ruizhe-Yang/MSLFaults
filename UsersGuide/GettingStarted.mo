within MSLFaults.UsersGuide;
class GettingStarted "Load the library and compare a first fault"
  extends Modelica.Icons.Information;
  annotation(Documentation(info="<html><p>Load Modelica 4.0.0, then open MSLFaults/package.mo. Open <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">Examples.ResistorReplacement</a> and simulate using its stored experiment settings.</p><p>Compare the native, normal and faulted current outputs. In the resistor path, 12 V across 10 Ω gives 1.2 A; the 20 Ω target gives 0.6 A at full effect. Inspect resistorFault.device.active and effectLevel to distinguish the requested interval from the transition ramp.</p></html>"));
end GettingStarted;
