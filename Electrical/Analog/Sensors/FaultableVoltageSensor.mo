within MSLFaults.Electrical.Analog.Sensors;
model FaultableVoltageSensor
  "Ideal voltage sensor with calibration, drift and hold faults"
  import SI = Modelica.Units.SI;
  extends Modelica.Icons.RoundSensor;

  Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive electrical pin"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n "Negative electrical pin"
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Blocks.Interfaces.RealOutput v(unit="V")
    "Fault-affected measured voltage"
    annotation (Placement(transformation(origin={0,-110}, extent={{10,-10},{-10,10}}, rotation=90)));

  type FaultMode = enumeration(
    Normal "Nominal behaviour",
    Bias "Additive measurement offset",
    Drift "Time-growing offset after fault onset",
    GainError "Measurement scale error",
    NoiseIncrease "Deterministic measurement interference",
    Stuck "Prescribed or captured measurement target",
    Dropout "Attenuate the measurement towards zero",
    Saturation "Symmetric measurement clipping");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter SI.Voltage biasFault=1 "Additive offset at full effect in the input or output unit";
  parameter Real gainFault=0.8 "Gain at full fault effect";
  parameter Real driftRate(unit="V/s")=0.1
    "Voltage drift rate at severity one";
  parameter SI.Voltage noiseAmplitude=0.01 "Amplitude of deterministic measurement interference";
  parameter SI.Frequency noiseFrequency=37 "Base frequency of deterministic interference";
  parameter Boolean useFixedStuckValue=false
    "Use stuckValue instead of the captured measurement";
  parameter SI.Voltage stuckValue=0 "Optional fixed measurement target at full effect";
  parameter SI.Voltage saturationLimit(min=Modelica.Constants.small)=1e6
    "Symmetric clipping magnitude in the measurement unit";

  SI.Voltage v_actual "True voltage before measurement faults";
  SI.Voltage stuckTarget "Current prescribed or captured measurement target";
  SI.Time activeElapsedTime "Duration of the current or most recently ended activation";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  discrete SI.Voltage stuckMemory(start=0,fixed=true)
    "True measurement captured at activation";


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
  activeElapsedTime=faultGate.elapsedActiveTime;

  p.i = 0;
  n.i = 0;
  v_actual = p.v - n.v;
  stuckTarget = if useFixedStuckValue then stuckValue else stuckMemory;

  when initial() then
    stuckMemory = v_actual;
  elsewhen active then
    stuckMemory = v_actual;
  end when;

  v = if faultMode == FaultMode.Bias then
      v_actual + faultActivation*biasFault
    elseif faultMode == FaultMode.Drift then
      v_actual + severity*endActivation*driftRate*activeElapsedTime
    elseif faultMode == FaultMode.GainError then
      v_actual*(1 + faultActivation*(gainFault - 1))
    elseif faultMode == FaultMode.NoiseIncrease then
      v_actual + faultActivation*noiseAmplitude*(
        0.65*sin(2*Modelica.Constants.pi*noiseFrequency*time) +
        0.35*sin(2*Modelica.Constants.pi*1.618*noiseFrequency*time))
    elseif faultMode == FaultMode.Stuck then
      v_actual + faultActivation*(stuckTarget-v_actual)
    elseif faultMode == FaultMode.Dropout then
      v_actual*(1-faultActivation)
    elseif faultMode == FaultMode.Saturation then
      v_actual + faultActivation*(min(saturationLimit,max(-saturationLimit,v_actual))-v_actual)
    else v_actual;



  annotation (
    Documentation(info="<html><p>The voltage measurement draws zero current from either pin. Faults alter only the output signal. Stuck captures the true measurement at each activation unless useFixedStuckValue selects stuckValue. NoiseIncrease adds a deterministic weighted sum of two sine waves. Bias, gain, dropout and clipping follow the activation envelope.</p><p>Connect p and n across the measured nodes; set the fault magnitude in V and compare v with v_actual.</p><p>Drift uses severity times elapsed active time. During recovery its offset remains at the deactivation value until effectLevel reaches zero, then vanishes.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Sensors.VoltageSensor</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-70,0},{-90,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{70,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{0,-100},{0,-70}}, color={0,0,127}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-30,-10},{30,-70}}, textString="V", textColor={64,64,64}),
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableVoltageSensor;
