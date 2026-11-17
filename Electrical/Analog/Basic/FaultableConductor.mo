within MSLFaults.Electrical.Analog.Basic;
model FaultableConductor "Conductor with conductance and temperature-coefficient faults"
  import SI = Modelica.Units.SI;
  parameter SI.Conductance G(start=1)
    "Conductance at temperature T_ref";
  parameter SI.Temperature T_ref=300.15 "Reference temperature";
  parameter SI.LinearTemperatureCoefficient alpha=0
    "Linear resistance temperature coefficient";
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=T_ref);
  SI.Conductance G_actual
    "Temperature-corrected conductance";

  type FaultMode = enumeration(Normal "Nominal behaviour", ConductanceDrift "Conductance change through the activation envelope", ConductanceStep "Conductance change through the activation envelope", OpenCircuit "Finite open-circuit equivalent", ShortCircuit "Finite short-circuit equivalent", TemperatureCoefficientDrift "Temperature-coefficient change");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter SI.Conductance G_fault=2*G "Reference conductance at full fault effect";
  parameter SI.Conductance G_open=1e-10 "Finite conductance representing an open circuit";
  parameter SI.Conductance G_short=1e6 "Finite conductance representing a short circuit";
  parameter SI.LinearTemperatureCoefficient alpha_fault=2*alpha "Temperature coefficient at full fault effect";
  SI.Conductance G_effective "Effective reference conductance";
  SI.LinearTemperatureCoefficient alpha_effective "Effective linear temperature coefficient";

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
  assert(G_open>0 and G_short>0 and (1+alpha*(T_heatPort-T_ref))>0,"Finite electrical targets must be positive.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  G_effective = if faultMode == FaultMode.ConductanceDrift or faultMode == FaultMode.ConductanceStep then G + faultActivation*(G_fault - G)
    elseif faultMode == FaultMode.OpenCircuit then G + faultActivation*(G_open - G)
    elseif faultMode == FaultMode.ShortCircuit then G + faultActivation*(G_short - G) else G;
  alpha_effective = if faultMode == FaultMode.TemperatureCoefficientDrift then alpha + faultActivation*(alpha_fault - alpha) else alpha;
  assert((1 + alpha_effective*(T_heatPort - T_ref)) >= Modelica.Constants.eps,
    "Temperature outside scope of model!");
  G_actual=G_effective/(1+alpha_effective*(T_heatPort-T_ref));
  i = G_actual*v;
  LossPower = v*i;

  assert(1+alpha_effective*(T_heatPort-T_ref)>0,"Electrical temperature correction must stay positive.");
  assert(G_actual>=0,"The passive electrical parameter must be nonnegative.");
  annotation (
    Documentation(info="<html><p>Modify the reference conductance or temperature coefficient, then divide by the linear temperature factor to obtain G_actual. OpenCircuit and ShortCircuit approach finite conductance targets; the temperature factor must remain positive.</p><p>Set G, G_fault or the finite open/short target; connect the electrical pins and optional heat port. Inspect G_actual and LossPower.</p><p>ConductanceDrift and ConductanceStep use the same activation-envelope interpolation; neither has a separate drift clock.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.Conductor</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Rectangle(extent={{-70,30},{70,-30}}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineColor={255,0,0}, lineThickness=0.75),
        Rectangle(extent={{-70,30},{70,-30}}, lineColor={0,0,255}, lineThickness=0.5),
        Line(points={{-90,0},{-70,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{70,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Line(visible=useHeatPort, points={{0,-100},{0,-30}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{-150,-40},{150,-80}}, textString="G=%G"),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableConductor;
