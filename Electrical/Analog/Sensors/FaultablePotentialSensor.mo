within MSLFaults.Electrical.Analog.Sensors;
model FaultablePotentialSensor "Ideal potential sensor with output faults"
  extends Modelica.Icons.RoundSensor;

  Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive electrical pin" annotation (Placement(
        transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Blocks.Interfaces.RealOutput phi(unit="V")
    "Absolute voltage potential as output signal"
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  type FaultMode = enumeration(Normal "Nominal behaviour", Bias "Additive measurement offset", GainError "Measurement scale error", NoiseIncrease "Deterministic measurement interference", Stuck "Prescribed or captured measurement target", Dropout "Attenuate the measurement towards zero", Saturation "Symmetric measurement clipping");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter Real biasFault=1 "Additive offset at full effect in the input or output unit";
  parameter Real gainFault=0.8 "Gain at full fault effect";
  parameter Real noiseAmplitude=0.01 "Amplitude of deterministic measurement interference";
  parameter Modelica.Units.SI.Frequency noiseFrequency=37 "Base frequency of deterministic interference";
  parameter Real stuckValue=0 "Prescribed value at full effect";
  parameter Real saturationLimit(min=Modelica.Constants.small)=1e6 "Symmetric clipping magnitude in the measurement unit";
  Real phi_actual "True electrical potential before measurement faults";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";

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

  p.i = 0;
  phi_actual = p.v;
  phi = if faultMode == FaultMode.Bias then phi_actual + faultActivation*biasFault
    elseif faultMode == FaultMode.GainError then phi_actual*(1 + faultActivation*(gainFault - 1))
    elseif faultMode == FaultMode.NoiseIncrease then phi_actual + faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time)
    elseif faultMode == FaultMode.Stuck then phi_actual + faultActivation*(stuckValue - phi_actual)
    elseif faultMode == FaultMode.Dropout then phi_actual*(1 - faultActivation)
    elseif faultMode == FaultMode.Saturation then phi_actual + faultActivation*(min(saturationLimit,max(-saturationLimit,phi_actual)) - phi_actual)
    else phi_actual;


  annotation (
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-70,0},{-90,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{100,0},{70,0}}, color={0,0,127}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-30,-10},{30,-70}}, textString="V", textColor={64,64,64}),
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>The single electrical pin draws zero current; phi_actual is its absolute potential. Bias, gain error, deterministic sinusoidal interference, prescribed stuck output, dropout and symmetric clipping modify only the output signal.</p><p>Connect p to the measured node and compare phi with phi_actual.</p><p>Stuck uses the fixed stuckValue in V; it does not capture the current measurement.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Sensors.PotentialSensor</code>.</p></html>"));
  
end FaultablePotentialSensor;
