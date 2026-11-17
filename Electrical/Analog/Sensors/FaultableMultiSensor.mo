within MSLFaults.Electrical.Analog.Sensors;
model FaultableMultiSensor "Current, voltage and power sensor with shared fault selection"
  extends Modelica.Icons.RoundSensor;
  Modelica.Electrical.Analog.Interfaces.PositivePin pc
      "Positive pin, current path"
    annotation (Placement(transformation(extent={{-90,-10},{-110,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin nc
      "Negative pin, current path"
    annotation (Placement(transformation(extent={{110,-10},{90,10}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin pv
      "Positive pin, voltage path"
    annotation (Placement(transformation(extent={{-10,110},{10,90}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin nv
      "Negative pin, voltage path"
    annotation (Placement(transformation(extent={{10,-110},{-10,-90}})));
  Modelica.Blocks.Interfaces.RealOutput i(final quantity="ElectricCurrent", final unit="A")
    "Current as output signal" annotation (Placement(transformation(
        origin={-60,-110},
        extent={{10,10},{-10,-10}},
        rotation=90)));
  Modelica.Blocks.Interfaces.RealOutput v(final quantity="ElectricPotential", final unit="V")
    "Voltage as output signal" annotation (Placement(transformation(
        origin={60,-110},
        extent={{10,10},{-10,-10}},
        rotation=90)));
  Modelica.Blocks.Interfaces.RealOutput power(final quantity="Power", final unit="W")
    "Instantaneous power as output signal"
    annotation (Placement(transformation(
          origin={-110,-60},
          extent={{-10,10},{10,-10}},
          rotation=180)));
  type FaultMode = enumeration(Normal "Nominal behaviour", Bias "Additive measurement offset", GainError "Measurement scale error", NoiseIncrease "Deterministic measurement interference", Stuck "Prescribed or captured measurement target", Dropout "Attenuate the measurement towards zero", Saturation "Symmetric measurement clipping");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter Real biasCurrent=0.1 "Current offset at full effect, in amperes";
  parameter Real biasVoltage=0.1 "Voltage offset at full effect, in volts";
  parameter Real biasPower=0.1 "Power offset at full effect, in watts";
  parameter Real gainFault=0.8 "Gain at full fault effect";
  parameter Real noiseAmplitude=0.01 "Amplitude of deterministic measurement interference";
  parameter Modelica.Units.SI.Frequency noiseFrequency=37 "Base frequency of deterministic interference";
  parameter Real stuckCurrent=0 "Prescribed current output at full effect, in amperes";
  parameter Real stuckVoltage=0 "Prescribed voltage output at full effect, in volts";
  parameter Real stuckPower=0 "Prescribed power output at full effect, in watts";
  parameter Real saturationLimit(min=Modelica.Constants.small)=1e6 "Symmetric clipping magnitude in the measurement unit";
  Real i_actual "True current before measurement faults";
  Real v_actual "True voltage before measurement faults";
  Real power_actual "True instantaneous power before measurement faults";
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

  pc.i + nc.i = 0;
  pc.v - nc.v = 0;
  pv.i = 0;
  nv.i = 0;
  i_actual = pc.i;
  v_actual = pv.v - nv.v;
  power_actual = v_actual*i_actual;
  i = if faultMode == FaultMode.Bias then i_actual + faultActivation*biasCurrent
    elseif faultMode == FaultMode.GainError then i_actual*(1 + faultActivation*(gainFault - 1))
    elseif faultMode == FaultMode.NoiseIncrease then i_actual + faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time)
    elseif faultMode == FaultMode.Stuck then i_actual + faultActivation*(stuckCurrent-i_actual)
    elseif faultMode == FaultMode.Dropout then i_actual*(1-faultActivation)
    elseif faultMode == FaultMode.Saturation then i_actual + faultActivation*(min(saturationLimit,max(-saturationLimit,i_actual))-i_actual) else i_actual;
  v = if faultMode == FaultMode.Bias then v_actual + faultActivation*biasVoltage
    elseif faultMode == FaultMode.GainError then v_actual*(1 + faultActivation*(gainFault - 1))
    elseif faultMode == FaultMode.NoiseIncrease then v_actual + faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time)
    elseif faultMode == FaultMode.Stuck then v_actual + faultActivation*(stuckVoltage-v_actual)
    elseif faultMode == FaultMode.Dropout then v_actual*(1-faultActivation)
    elseif faultMode == FaultMode.Saturation then v_actual + faultActivation*(min(saturationLimit,max(-saturationLimit,v_actual))-v_actual) else v_actual;
  power = if faultMode == FaultMode.Bias then power_actual + faultActivation*biasPower
    elseif faultMode == FaultMode.GainError then power_actual*(1 + faultActivation*(gainFault - 1))
    elseif faultMode == FaultMode.NoiseIncrease then power_actual + faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time)
    elseif faultMode == FaultMode.Stuck then power_actual + faultActivation*(stuckPower-power_actual)
    elseif faultMode == FaultMode.Dropout then power_actual*(1-faultActivation)
    elseif faultMode == FaultMode.Saturation then power_actual + faultActivation*(min(saturationLimit,max(-saturationLimit,power_actual))-power_actual) else power_actual;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{0,100},{0,70}}, color={0,0,255}, thickness=0.5),
        Line(points={{0,-70},{0,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-100,0},{100,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{0,70},{0,40}}, color={64,64,64}, thickness=0.5),
        Line(points={{-100,-60},{-80,-60},{-56,-42}}, color={0,0,127}, thickness=0.5),
        Line(points={{-60,-100},{-60,-80},{-42,-56}}, color={0,0,127}, thickness=0.5),
        Line(points={{60,-100},{60,-80},{42,-56}}, color={0,0,127}, thickness=0.5),
        Text(extent={{-100,-40},{-60,-80}}, textColor={64,64,64}, textString="W"),
        Text(extent={{-80,-60},{-40,-100}}, textColor={64,64,64}, textString="A"),
        Text(extent={{40,-60},{80,-100}}, textColor={64,64,64}, textString="V"),
        Text(textColor={0,0,255}, extent={{-150,120},{150,160}}, textString="%name"),
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>Measure current through pc/nc and voltage across pv/nv without changing their ideal sensor relations. Apply the same selected fault mode independently to i, v and power, using channel-specific bias and stuck values. NoiseIncrease is deterministic sinusoidal interference.</p><p>Connect both measurement paths; configure biasCurrent/biasVoltage/biasPower or the corresponding stuck targets and inspect all three outputs.</p><p>The faulted power output need not equal the product of the faulted current and voltage. noiseAmplitude and saturationLimit are shared numerical values interpreted in each channel unit.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Sensors.MultiSensor</code>.</p></html>"));
end FaultableMultiSensor;
