within MSLFaults.Electrical.Analog.Basic;
model FaultablePotentiometer "Potentiometer with track and wiper faults"
  import SI = Modelica.Units.SI;
  parameter SI.Resistance R(start=1)
    "Resistance at temperature T_ref";
  parameter SI.Temperature T_ref=293.15 "Reference temperature";
  parameter SI.LinearTemperatureCoefficient alpha=0
    "Linear resistance temperature coefficient";
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=T_ref);
  parameter Boolean useRinput=false "Use input r for the wiper position"
    annotation (
    Evaluate=true,
    HideResult=true,
    Dialog(group="potentiometer"));
  parameter Real rConstant(
    final min=0,
    final max=1) = 0.5 "Fixed relative wiper position, from n at zero to p at one"
    annotation (Dialog(group="potentiometer", enable=not useRinput));
  SI.Resistance Rp
    "Actual resistance between pin_p and contact";
  SI.Resistance Rn
    "Actual resistance between contact and pin_n";
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
    "Positive electrical terminal" annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin contact "Wiper terminal" annotation (
      Placement(transformation(extent={{90,-110},{110,-90}}),
        iconTransformation(extent={{90,-110},{110,-90}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
    "Negative electrical terminal" annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Blocks.Interfaces.RealInput r if useRinput "Commanded relative wiper position" annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-100,-120}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-100,-120})));
  type FaultMode = enumeration(Normal "Nominal behaviour", TotalResistanceDrift "Total track-resistance change", WiperBias "Wiper-position offset", WiperStuck "Prescribed wiper position", ContactOpen "Finite contact-resistance increase");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter SI.Resistance R_fault=2*R "Total track resistance at full effect";
  parameter Real wiperBias=0.1 "Wiper-position offset at full effect";
  parameter Real wiperStuckValue(min=0,max=1)=0.5 "Prescribed wiper position at full effect";
  parameter SI.Resistance R_contactOpen=1e10 "Finite resistance added to each contact branch";
  SI.Resistance R_effective "Effective total track resistance";
  Real r_effective "Effective relative wiper position";
  SI.Resistance R_contact "Added resistance in each terminal-to-contact branch";
protected
  Modelica.Blocks.Sources.Constant rConst(final k=rConstant) if not useRinput
    "Fixed relative wiper-position source" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-90,-50})));
  Modelica.Blocks.Interfaces.RealInput rInt
    "Internal relative wiper-position command" annotation (Placement(transformation(extent={{-84,-84},{-76,-76}})));

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
  pin_p.i + pin_n.i + contact.i = 0;
  R_effective = if faultMode == FaultMode.TotalResistanceDrift then R + faultActivation*(R_fault - R) else R;
  r_effective = if faultMode == FaultMode.WiperBias then min(1, max(0, rInt + faultActivation*wiperBias))
    elseif faultMode == FaultMode.WiperStuck then rInt + faultActivation*(wiperStuckValue - rInt) else min(1, max(0, rInt));
  R_contact = if faultMode == FaultMode.ContactOpen then faultActivation*R_contactOpen else 0;
  Rp = R_effective*(1 + alpha*(T_heatPort - T_ref))*(1 - r_effective);
  Rn = R_effective*(1 + alpha*(T_heatPort - T_ref))*r_effective;
  pin_p.v - contact.v = (Rp + R_contact)*pin_p.i;
  pin_n.v - contact.v = (Rn + R_contact)*pin_n.i;
  LossPower = (pin_p.v - contact.v)*pin_p.i + (pin_n.v - contact.v)*pin_n.i;
  connect(rInt, r) annotation (Line(
      points={{-80,-80},{-100,-80},{-100,-120}}, color={0,0,127}));
  connect(rInt, rConst.y) annotation (Line(
      points={{-80,-80},{-90,-80},{-90,-61}}, color={0,0,127}));


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}),
      graphics={
        Rectangle(extent={{-70,30},{70,-30}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{-90,0},{-70,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{70,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Line(visible=useHeatPort, points={{0,-100},{0,-30}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Line(points={{0,40},{0,-40},{100,-80},{100,-90}}, color={0,0,255}, thickness=0.5),
        Line(visible=useRinput, points={{-100,-90},{-100,-80},{0,-40}}, color={0,0,255}, pattern=LinePattern.Dot, thickness=0.5),
        Line(visible=useHeatPort, points={{0,-90},{0,-40}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Polygon(points={{0,-30},{-4,-40},{4,-40},{0,-30}}, lineColor={0,0,255}, fillColor={0,0,255}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>Split the temperature-corrected track resistance according to rConstant or input r. Faults modify total resistance, bias the wiper, interpolate towards wiperStuckValue, or add R_contactOpen to both terminal-to-contact equations.</p><p>Connect pin_p, pin_n and contact; select useRinput for a commanded wiper and inspect r_effective, Rp, Rn and LossPower.</p><p>ContactOpen is a finite resistance added to both track branches. WiperStuck uses a configured position rather than a captured position.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.Potentiometer</code>.</p></html>"));
end FaultablePotentiometer;
