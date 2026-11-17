within MSLFaults.Electrical.Analog.Basic;
model FaultableResistor
  "Resistor with resistance and temperature-coefficient faults"
  import SI = Modelica.Units.SI;

  parameter SI.Resistance R(start=1) "Resistance at reference temperature";
  parameter SI.Temperature T_ref=300.15 "Reference temperature";
  parameter SI.LinearTemperatureCoefficient alpha=0
    "Linear resistance temperature coefficient";

  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=T_ref);

  type FaultMode = enumeration(
      Normal "Nominal behaviour",
      ResistanceDrift "Progressive resistance drift",
      ResistanceStep "Abrupt resistance change",
      OpenCircuit "Finite open-circuit equivalent",
      ShortCircuit "Finite short-circuit equivalent",
      TemperatureCoefficientDrift "Temperature-coefficient change");

  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0, max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1
    "Time to reach the full progressive-drift target";
  parameter SI.Resistance R_drift=2*R "Resistance at completed drift";
  parameter SI.Resistance R_step=2*R "Resistance after an abrupt step";
  parameter SI.Resistance R_open(min=Modelica.Constants.small)=1e10
    "Finite resistance representing an open circuit";
  parameter SI.Resistance R_short(min=Modelica.Constants.small)=1e-6
    "Finite resistance representing a short circuit";
  parameter SI.LinearTemperatureCoefficient alpha_fault=2*alpha
    "Temperature coefficient at full fault effect";

  SI.Resistance R_actual
    "Temperature-corrected resistance";
  SI.Resistance R_effective "Effective reference resistance";
  SI.LinearTemperatureCoefficient alpha_effective
    "Effective linear temperature coefficient";
  Real faultActivation(min=0, max=1) "Severity-scaled fault envelope";
  Real driftActivation(min=0, max=1) "Fault envelope multiplied by progressive drift";
  Real startActivation(min=0, max=1) "Activation envelope divided by severity";
  Real endActivation(min=0, max=1) "One while active or recovering, otherwise zero";
  Real driftProgress(min=0, max=1) "Elapsed activation time divided by driftTime, limited to one";


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
initial equation
  assert(R_open>0 and R_short>0 and (1+alpha*(T_heatPort-T_ref))>0,"Finite electrical targets must be positive.");
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

  R_effective = if faultMode == FaultMode.ResistanceDrift then
      R + driftActivation*(R_drift - R)
    elseif faultMode == FaultMode.ResistanceStep then
      R + faultActivation*(R_step - R)
    elseif faultMode == FaultMode.OpenCircuit then
      R + faultActivation*(R_open - R)
    elseif faultMode == FaultMode.ShortCircuit then
      R + faultActivation*(R_short - R)
    else R;
  alpha_effective = if faultMode == FaultMode.TemperatureCoefficientDrift then
      alpha + driftActivation*(alpha_fault - alpha) else alpha;

  assert((1 + alpha_effective*(T_heatPort - T_ref)) >= Modelica.Constants.eps,
    "Temperature outside the valid linear resistance model range");
  R_actual=R_effective*(1+alpha_effective*(T_heatPort-T_ref));
  v = R_actual*i;
  LossPower = v*i;


  assert(1+alpha_effective*(T_heatPort-T_ref)>0,"Electrical temperature correction must stay positive.");
  assert(R_actual>=0,"The passive electrical parameter must be nonnegative.");
  annotation (
    Documentation(info="<html><p>The reference resistance is modified before applying the linear temperature correction. ResistanceDrift and TemperatureCoefficientDrift combine the activation envelope with elapsed time divided by driftTime; ResistanceStep, OpenCircuit and ShortCircuit use the envelope directly. Open and short circuits use finite resistance targets.</p><p>Set R and the selected resistance or alpha target; connect p and n and, when enabled, heatPort. Inspect R_effective, R_actual and LossPower.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.Resistor</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-70,30},{70,-30}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{-90,0},{-70,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{70,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,-40},{150,-80}}, textString="R=%R"),
        Line(visible=useHeatPort, points={{0,-100},{0,-30}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableResistor;
