within MSLFaults.Electrical.Analog.Ideal;
model FaultableIdealOpeningSwitch "Boolean-controlled opening switch with contact faults"
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  parameter Modelica.Units.SI.Resistance Ron(final min=0)=1e-5 "Resistance in the closed state";
  parameter Modelica.Units.SI.Conductance Goff(final min=0)=1e-5 "Conductance in the open state";
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=293.15);
  Modelica.Blocks.Interfaces.BooleanInput control annotation(Placement(transformation(origin={0,120},extent={{-20,-20},{20,20}},rotation=270)));
  type FaultMode=enumeration(Normal "Nominal behaviour", StuckOpen "Force the open relation above the effect threshold", StuckClosed "Force the closed relation above the effect threshold", ContactResistanceIncrease "Increased closed-state resistance", LeakageIncrease "Increased leakage conductance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter Modelica.Units.SI.Resistance RonFault=1 "Closed-state resistance at full fault effect";
  parameter Modelica.Units.SI.Conductance GoffFault=1e-2 "Open-state conductance at full fault effect";
  Boolean off "Effective open-state selection";
  Boolean naturalOff "Open-state selection without forced-state faults";
  Real s(final unit="1");
  constant Modelica.Units.SI.Voltage unitVoltage=1 "Voltage scale for the normalised characteristic";
  constant Modelica.Units.SI.Current unitCurrent=1 "Current scale for the normalised characteristic";
  Modelica.Units.SI.Resistance Ron_effective "Effective closed-state resistance";
  Modelica.Units.SI.Conductance Goff_effective "Effective open-state conductance";

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
  naturalOff=control;
  off=if faultMode==FaultMode.StuckOpen and faultActivation>0.5 then true elseif faultMode==FaultMode.StuckClosed and faultActivation>0.5 then false else naturalOff;
  Ron_effective=if faultMode==FaultMode.ContactResistanceIncrease then Ron+faultActivation*(RonFault-Ron) else Ron;
  Goff_effective=if faultMode==FaultMode.LeakageIncrease then Goff+faultActivation*(GoffFault-Goff) else Goff;
  v=(s*unitCurrent)*(if off then 1 else Ron_effective);
  i=(s*unitVoltage)*(if off then Goff_effective else 1);
  LossPower=v*i;


  annotation (defaultComponentName="switch",
    Documentation(info="<html><p>The nominal switch is open when control. StuckOpen and StuckClosed override that state; ContactResistanceIncrease changes Ron and LeakageIncrease changes Goff. The optional heat port receives v*i.</p><p>Connect p, n and control; set RonFault or GoffFault for parametric faults and inspect off and LossPower.</p><p>Forced switch positions require effectLevel&gt;0.5; at severity&lt;=0.5 they are never selected. Resistance and leakage changes follow the continuous envelope.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(visible=useHeatPort, points={{0,-100},{0,25}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Line(points={{-90,0},{-40,0}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{40,0},{90,0}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Ellipse(extent={{-44,-4},{-36,4}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.7),
        Ellipse(extent={{36,-4},{44,4}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.7),
        Line(points={{-37,2},{37,34}}, color={255,0,0}, thickness=1, pattern=LinePattern.Solid),
        Line(points={{13,27},{13,52}}, color={64,64,64}, thickness=0.6, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-150,185},{150,155}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealOpeningSwitch;
