within MSLFaults.Electrical.Analog.Basic;
model FaultableInductor
  "Inductor with inductance, winding-loss and turn-short faults"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort(i(start=0));
  parameter SI.Inductance L(start=1) "Inductance";

  type FaultMode = enumeration(
    Normal "Nominal behaviour",
    InductanceDrift "Progressive inductance drift",
    InductanceLoss "Interpolate towards the inductance-loss target",
    WindingResistanceIncrease "Series winding resistance increase",
    TurnShort "Reduced effective turns plus additional winding loss",
    OpenCircuit "Finite open-circuit equivalent");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1 "Time to reach the full progressive-drift target";
  parameter SI.Inductance L_drift=1.2*L "Inductance at completed progressive drift";
  parameter SI.Inductance L_loss=0.5*L "Inductance target for the loss mode";
  parameter SI.Resistance R_windingFault=10 "Additional winding resistance at full effect";
  parameter Real maxTurnShortRatio(min=0,max=0.95)=0.3
    "Shorted-turn fraction at full effect";
  parameter SI.Resistance R_turnShortLoss=1
    "Additional series loss resistance at full TurnShort effect";
  parameter SI.Resistance R_open=1e10 "Finite resistance representing an open circuit";

  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real driftActivation(min=0,max=1) "Fault envelope multiplied by progressive drift";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Real driftProgress(min=0,max=1) "Elapsed activation time divided by driftTime, limited to one";
  Real turnShortRatio(min=0,max=0.95) "Effective shorted-turn fraction";
  SI.Inductance L_effective "Effective inductance";
  SI.Resistance R_series "Fault-adjusted winding loss/open resistance";


public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the scheduled fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    "External fault activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Fault activation request state";
  output Real effectLevel(min=0,max=1) "Severity-scaled fault envelope";
  output Integer activationCount "Number of activation starts";
  Modelica.Units.SI.MagneticFlux psi "Flux linkage";
  Modelica.Units.SI.Energy storedEnergy "Stored electrical or magnetic energy";
  Modelica.Units.SI.Power lossPower "Electrical power dissipated in resistive branches",parameterPower "Power associated with prescribed storage-parameter variation";
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger) "Reversible fault activation envelope";
initial equation
  assert(L>0 and L_loss>0 and L_drift>0 and driftTime>0,"Inductances and drift time must be positive.");
  assert(R_windingFault>=0 and R_turnShortLoss>=0 and R_open>0 and maxTurnShortRatio>=0 and maxTurnShortRatio<1,"Invalid winding-loss targets.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  driftProgress=min(1,max(0,faultGate.elapsedActiveTime/driftTime));
  driftActivation=effectLevel*driftProgress;
  turnShortRatio = if faultMode == FaultMode.TurnShort then
      faultActivation*maxTurnShortRatio else 0;

  L_effective = if faultMode == FaultMode.InductanceDrift then
      L + driftActivation*(L_drift-L)
    elseif faultMode == FaultMode.InductanceLoss then
      L + faultActivation*(L_loss-L)
    elseif faultMode == FaultMode.TurnShort then
      L*(1-turnShortRatio)^2 else L;
  R_series = if faultMode == FaultMode.WindingResistanceIncrease then
      faultActivation*R_windingFault
    elseif faultMode == FaultMode.TurnShort then
      faultActivation*R_turnShortLoss
    elseif faultMode == FaultMode.OpenCircuit then
      faultActivation*R_open else 0;

  v=der(psi)+R_series*i;


  psi=L_effective*i;
  storedEnergy=psi*psi/(2*L_effective);
  lossPower=R_series*i*i;
  parameterPower=der(storedEnergy)-v*i+lossPower;
  annotation (
    Documentation(info="<html><p>Evolve flux linkage psi with psi=L_effective*i and v=der(psi)+R_series*i. InductanceDrift uses driftTime; InductanceLoss follows the activation envelope. TurnShort reduces inductance with the square of the remaining turn fraction and adds series loss. OpenCircuit adds a finite high resistance.</p><p>Set L and the selected target; for TurnShort configure maxTurnShortRatio and R_turnShortLoss. Inspect psi, storedEnergy, lossPower and parameterPower.</p><p>TurnShort is an equivalent winding model. parameterPower records the power associated with prescribed inductance variation.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.Inductor</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{60,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{-90,0},{-60,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,-40},{150,-80}}, textString="L=%L"),
        Line(points={{-60,0},{-59,6},{-52,14},{-38,14},{-31,6},{-30,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{-30,0},{-29,6},{-22,14},{-8,14},{-1,6},{0,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{0,0},{1,6},{8,14},{22,14},{29,6},{30,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{30,0},{31,6},{38,14},{52,14},{59,6},{60,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableInductor;
