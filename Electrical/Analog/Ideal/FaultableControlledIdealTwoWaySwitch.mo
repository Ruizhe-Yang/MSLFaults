within MSLFaults.Electrical.Analog.Ideal;
model FaultableControlledIdealTwoWaySwitch "Voltage-controlled changeover switch with contact faults"
  parameter Modelica.Units.SI.Voltage level=0.5 "Switch level";
  parameter Modelica.Units.SI.Resistance Ron(final min=0) = 1e-5 "Resistance in the closed state";
  parameter Modelica.Units.SI.Conductance Goff(final min=0) = 1e-5
    "Conductance in the open state";
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
        293.15);
  Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive electrical pin" annotation (Placement(transformation(extent={{-110,
            -10},{-90,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n2 annotation (Placement(transformation(extent={{90,
            -10},{110,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1 annotation (Placement(transformation(extent={{90,30},{110,50}}), iconTransformation(extent={{90,30},{110,50}})));
  Modelica.Electrical.Analog.Interfaces.Pin control
    "Control pin: if control.v > level p--n2 connected, otherwise p--n1 connected"
    annotation (Placement(transformation(
        origin={0,100},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  type FaultMode=enumeration(Normal "Nominal behaviour", StuckPosition1 "Force the first routing above the effect threshold", StuckPosition2 "Force the second routing above the effect threshold", ContactResistanceIncrease "Increased closed-state resistance", LeakageIncrease "Increased leakage conductance");
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
  Boolean control_effective "Effective routing selection";
  Boolean control_command "Routing selection from the control input";
  Modelica.Units.SI.Resistance Ron_effective "Effective closed-state resistance";
  Modelica.Units.SI.Conductance Goff_effective "Effective open-state conductance";
protected
  Real s1(final unit="1");
  Real s2(final unit="1") "Auxiliary variables";
  constant Modelica.Units.SI.Voltage unitVoltage=1 "Voltage scale for the normalised characteristic" annotation (HideResult=true);
  constant Modelica.Units.SI.Current unitCurrent=1 "Current scale for the normalised characteristic" annotation (HideResult=true);

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
  control_command=control.v > level;
  control.i=0;
  control_effective=if faultMode==FaultMode.StuckPosition1 and faultActivation>0.5 then false elseif faultMode==FaultMode.StuckPosition2 and faultActivation>0.5 then true else control_command;
  Ron_effective=if faultMode==FaultMode.ContactResistanceIncrease then Ron+faultActivation*(RonFault-Ron) else Ron;
  Goff_effective=if faultMode==FaultMode.LeakageIncrease then Goff+faultActivation*(GoffFault-Goff) else Goff;

  0 = p.i + n2.i + n1.i;

  p.v - n1.v = (s1*unitCurrent)*(if (control_effective) then 1 else Ron_effective);
  n1.i = -(s1*unitVoltage)*(if (control_effective) then Goff_effective else 1);
  p.v - n2.v = (s2*unitCurrent)*(if (control_effective) then Ron_effective else 1);
  n2.i = -(s2*unitVoltage)*(if (control_effective) then 1 else Goff_effective);
  LossPower = p.i*p.v + n1.i*n1.v + n2.i*n2.v;


  annotation (defaultComponentName="switch",
    Documentation(info="<html><p>When control.v&gt;level, connect p to n2; otherwise connect p to n1. StuckPosition1 selects the false routing and StuckPosition2 the true routing. ContactResistanceIncrease and LeakageIncrease modify the common branch parameters.</p><p>Connect every switching terminal and the control input; inspect control_effective, Ron_effective, Goff_effective and LossPower.</p><p>Forced switch positions require effectLevel&gt;0.5; at severity&lt;=0.5 they are never selected. Resistance and leakage changes follow the continuous envelope.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Ideal.ControlledIdealTwoWaySwitch</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Ellipse(extent={{-44,4},{-36,-4}}, lineColor={0,0,255}, lineThickness=0.5),
        Line(points={{-90,0},{-44,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{-37,2},{40,40}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,40},{90,40}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Line(visible=useHeatPort, points={{0,-99},{0,25}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableControlledIdealTwoWaySwitch;
