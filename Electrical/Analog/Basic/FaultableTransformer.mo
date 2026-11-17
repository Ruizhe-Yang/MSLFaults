within MSLFaults.Electrical.Analog.Basic;
model FaultableTransformer "Coupled-inductor transformer with winding faults"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.TwoPort(i1(start=0),i2(start=0));
  parameter SI.Inductance L1(start=1) "Primary inductance";
  parameter SI.Inductance L2(start=1) "Secondary inductance";
  parameter SI.Inductance M(start=1) "Coupling inductance";
  Real dv "Difference between voltage drop over primary inductor and voltage drop over secondary inductor";
  type FaultMode = enumeration(Normal "Nominal behaviour", PrimaryInductanceDrift "Primary inductance change", SecondaryInductanceDrift "Secondary inductance change", CouplingLoss "Reduced coupling", PrimaryOpen "Finite primary winding resistance", SecondaryOpen "Finite secondary winding resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter SI.Inductance L1_fault=0.5*L1 "Primary inductance at full effect";
  parameter SI.Inductance L2_fault=0.5*L2 "Secondary inductance at full effect";
  parameter SI.Inductance M_fault=0.1*M "Mutual inductance at full effect";
  parameter SI.Resistance R_open=1e10 "Finite resistance representing an open circuit";
  SI.Inductance L1_effective "Effective primary inductance";
  SI.Inductance L2_effective "Effective secondary inductance";
  SI.Inductance M_effective "Effective mutual inductance";
  SI.Resistance R1_series "Additional primary series resistance";
  SI.Resistance R2_series "Additional secondary series resistance";

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
  L1_effective = if faultMode == FaultMode.PrimaryInductanceDrift then L1 + faultActivation*(L1_fault - L1) else L1;
  L2_effective = if faultMode == FaultMode.SecondaryInductanceDrift then L2 + faultActivation*(L2_fault - L2) else L2;
  M_effective = if faultMode == FaultMode.CouplingLoss then M + faultActivation*(M_fault - M) else M;
  R1_series = if faultMode == FaultMode.PrimaryOpen then faultActivation*R_open else 0;
  R2_series = if faultMode == FaultMode.SecondaryOpen then faultActivation*R_open else 0;
  v1 = L1_effective*der(i1) + M_effective*der(i2) + R1_series*i1;
  v2 = M_effective*der(i1) + L2_effective*der(i2) + R2_series*i2;
  dv = v1 - v2;



  annotation (
    Documentation(info="<html><p>Modify primary inductance, secondary inductance or mutual inductance in the coupled current-derivative equations. PrimaryOpen and SecondaryOpen add finite series resistance to the corresponding winding.</p><p>Set L1, L2 and M consistently, then configure the selected inductance target or R_open. Inspect winding currents and effective parameters.</p><p>The equations multiply current derivatives by the varying inductances; they do not differentiate a time-varying flux-linkage matrix.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.Transformer</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-20,-60},{20,-100}}, textString="M", textColor={0,0,255}),
        Line(points={{-40,60},{-40,100},{-90,100}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,60},{40,100},{90,100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-40,-60},{-40,-100},{-90,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,-60},{40,-100},{90,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,45}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,15}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,-15}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,-45}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,45}, rotation=90, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,15}, rotation=90, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,-15}, rotation=90, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,-45}, rotation=90, thickness=0.5),
        Text(extent={{-100,20},{-58,-20}}, textString="L1", textColor={0,0,255}),
        Text(extent={{60,20},{100,-20}}, textString="L2", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableTransformer;
