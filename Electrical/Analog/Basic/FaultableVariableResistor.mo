within MSLFaults.Electrical.Analog.Basic;
model FaultableVariableResistor
  "Variable resistance with input and finite open/short faults"
  import SI = Modelica.Units.SI;
  parameter SI.Temperature T_ref=300.15 "Reference temperature";
  parameter SI.LinearTemperatureCoefficient alpha=0
    "Linear resistance temperature coefficient";
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=T_ref);
  SI.Resistance R_actual
    "Temperature-corrected resistance";
  Modelica.Blocks.Interfaces.RealInput R(unit="Ohm") "Commanded reference resistance" annotation (Placement(
        transformation(
        origin={0,120},
        extent={{-20,-20},{20,20}},
        rotation=270), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,120})));
  type FaultMode = enumeration(Normal "Nominal behaviour", ResistanceScaleError "Scale the commanded resistance", ResistanceBias "Bias the commanded resistance", OpenCircuit "Finite open-circuit equivalent", ShortCircuit "Finite short-circuit equivalent");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter Real scaleFault=0.5 "Input scale factor at full effect";
  parameter SI.Resistance biasFault=1 "Additive offset at full effect in the input or output unit";
  parameter SI.Resistance R_open=1e10 "Finite resistance representing an open circuit";
  parameter SI.Resistance R_short=1e-6 "Finite resistance representing a short circuit";
  SI.Resistance R_effective "Effective reference resistance";

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
  assert((1 + alpha*(T_heatPort - T_ref)) >= Modelica.Constants.eps,
    "Temperature outside scope of model!");
  R_effective = if faultMode == FaultMode.ResistanceScaleError then R*(1 + faultActivation*(scaleFault - 1))
    elseif faultMode == FaultMode.ResistanceBias then R + faultActivation*biasFault
    elseif faultMode == FaultMode.OpenCircuit then R + faultActivation*(R_open - R)
    elseif faultMode == FaultMode.ShortCircuit then R + faultActivation*(R_short - R) else R;
  R_actual = R_effective*(1 + alpha*(T_heatPort - T_ref));
  v = R_actual*i;
  LossPower = v*i;


  annotation (defaultComponentName="resistor",
    Documentation(info="<html><p>Transform the commanded resistance by a scale error, additive bias or interpolation towards the finite open/short target. Apply the nominal linear temperature correction to obtain R_actual; the electrical loss is v*i.</p><p>Connect input R and the electrical pins; set scaleFault, biasFault or the corresponding finite target. Inspect R_effective and R_actual.</p><p>Choose targets that retain the intended passive behaviour and a positive temperature correction; the fault command is not clamped to a nonnegative value.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.VariableResistor</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{-90,0},{-70,0}}, color={0,0,255}, thickness=0.5),
        Rectangle(extent={{-70,30},{70,-30}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{70,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,185},{150,155}}, textString="%name", textColor={0,0,255}),
        Line(points={{-45,-42},{44,45}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-32,-55},{32,-84}}, textString="R", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableVariableResistor;
