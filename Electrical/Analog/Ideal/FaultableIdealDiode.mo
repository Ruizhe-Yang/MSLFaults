within MSLFaults.Electrical.Analog.Ideal;
model FaultableIdealDiode "Piecewise linear diode with conduction faults"
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  parameter Modelica.Units.SI.Resistance Ron(final min=0)=1e-5 "Resistance in the closed state";
  parameter Modelica.Units.SI.Conductance Goff(final min=0)=1e-5 "Conductance in the open state";
  parameter Modelica.Units.SI.Voltage Vknee(final min=0)=0 "Nominal forward knee voltage";
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort;
  type FaultMode=enumeration(Normal "Nominal behaviour", OpenCircuit "Finite open-circuit equivalent", ShortCircuit "Finite short-circuit equivalent", ForwardVoltageDrift "Forward-conduction voltage change", LeakageIncrease "Increased leakage conductance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter Modelica.Units.SI.Resistance R_short=1e-6 "Finite resistance representing a short circuit";
  parameter Modelica.Units.SI.Conductance G_open=1e-10 "Finite conductance representing an open circuit";
  parameter Modelica.Units.SI.Conductance G_leakFault=1e-2 "Leakage conductance at full effect";
  parameter Modelica.Units.SI.Voltage VkneeFault=1 "Forward knee voltage at full effect";
  Boolean off(start=true) "Effective open-state selection";
  Boolean naturalOff "Open-state selection without forced-state faults";
  Real s(start=0,final unit="1");
  constant Modelica.Units.SI.Voltage unitVoltage=1 "Voltage scale for the normalised characteristic";
  constant Modelica.Units.SI.Current unitCurrent=1 "Current scale for the normalised characteristic";
  Modelica.Units.SI.Resistance Ron_effective "Effective closed-state resistance";
  Modelica.Units.SI.Conductance Goff_effective "Effective open-state conductance";
  Modelica.Units.SI.Voltage Vknee_effective "Effective forward knee voltage";

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the scheduled fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    "External fault activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Fault activation request state";
  output Real effectLevel(min=0,max=1) "Severity-scaled fault envelope";
  output Integer activationCount "Number of activation starts";

protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger) "Reversible fault activation envelope";
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  naturalOff = s < 0;
  off = if faultMode==FaultMode.OpenCircuit and faultActivation>0.5 then true
    elseif faultMode==FaultMode.ShortCircuit and faultActivation>0.5 then false else naturalOff;
  Ron_effective = if faultMode==FaultMode.ShortCircuit then Ron+faultActivation*(R_short-Ron) else Ron;
  Goff_effective = if faultMode==FaultMode.OpenCircuit then Goff+faultActivation*(G_open-Goff)
    elseif faultMode==FaultMode.LeakageIncrease then Goff+faultActivation*(G_leakFault-Goff) else Goff;
  Vknee_effective = if faultMode==FaultMode.ForwardVoltageDrift then Vknee+faultActivation*(VkneeFault-Vknee) else Vknee;
  v=(s*unitCurrent)*(if off then 1 else Ron_effective)+Vknee_effective;
  i=(s*unitVoltage)*(if off then Goff_effective else 1)+Goff_effective*Vknee_effective;
  LossPower=v*i;


  annotation(
    Documentation(info="<html><p>Retain a finite on-resistance, off-conductance and forward knee voltage. OpenCircuit forces the off relation once effectLevel&gt;0.5 and moves Goff towards G_open; ShortCircuit similarly forces the on relation and moves Ron towards R_short. Other modes modify Vknee or off-state leakage.</p><p>Connect p and n, configure the finite targets, and inspect off, the effective parameters and LossPower.</p><p>Forced on/off states require effectLevel&gt;0.5. ShortCircuit retains the configured knee voltage, so its terminal relation is not generally a pure resistor.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Ideal.IdealDiode</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{30,0},{-30,40},{-30,-40},{30,0}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{-90,0},{40,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{30,40},{30,-40}}, color={255,0,0}, thickness=0.8),
        Line(visible=useHeatPort, points={{0,-100},{0,-20}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealDiode;
