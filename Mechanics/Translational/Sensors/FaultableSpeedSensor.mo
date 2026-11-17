within MSLFaults.Mechanics.Translational.Sensors;
model FaultableSpeedSensor "Absolute velocity sensor with configurable output faults"
  extends Modelica.Mechanics.Translational.Interfaces.PartialAbsoluteSensor;
  Modelica.Blocks.Interfaces.RealOutput v(unit="m/s") "Reported velocity" annotation(Placement(transformation(origin={0,-110},extent={{10,-10},{-10,10}},rotation=90)));
  type FaultMode=enumeration(Normal "Nominal component relation", Bias "Additive measurement offset", GainError "Changed signal gain", NoiseIncrease "Deterministic sinusoidal measurement disturbance", Stuck "Blend towards a prescribed constant", Dropout "Attenuate the measurement towards zero", Saturation "Unconditional symmetric clipping while this mode is selected");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  parameter Real biasFault=1 "Offset amplitude in the measurement unit";
  parameter Real gainFault=0.5 "Measurement gain at full effect";
  parameter Real stuckValue=0 "Prescribed output at full stuck effect";
  parameter Real saturationLimit=1e9 "Symmetric clipping magnitude in the measurement unit";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law",startActivation(min=0,max=1) "Activation envelope divided by positive severity",endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  Real v_raw "Unmodified measurement before applying the selected signal fault",v_faulted "Measurement after the selected signal fault";

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";

protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  v_raw=der(flange.s);
  v_faulted=if faultMode==FaultMode.Bias then v_raw+faultActivation*biasFault elseif faultMode==FaultMode.GainError then v_raw*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.NoiseIncrease then v_raw+faultActivation*biasFault*sin(997*time) elseif faultMode==FaultMode.Stuck then v_raw+faultActivation*(stuckValue-v_raw) elseif faultMode==FaultMode.Dropout then (1-faultActivation)*v_raw elseif faultMode==FaultMode.Saturation then min(saturationLimit,max(-saturationLimit,v_raw)) else v_raw;
  v=v_faulted;


  annotation(
    Documentation(info="<html><p>The unmodified reading follows v_raw=der(flange.s). Bias and GainError apply an additive or multiplicative change. Stuck blends towards the specified stuckValue; Dropout blends towards zero. NoiseIncrease adds the deterministic sinusoid effectLevel*biasFault*sin(997*time).</p><p>Connect flange; read v in m/s and configure biasFault, gainFault, stuckValue or saturationLimit.</p><p>Saturation clips directly to +/-saturationLimit whenever that mode is selected, independent of enabled, severity, the time window and trigger. Use Normal for an unmodified baseline; other modes use the activation envelope.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Sensors.SpeedSensor</code>. Inspect the reported signal together with <code>active</code> and <code>effectLevel</code>.</p></html>"),
    defaultComponentName="speedSensor",Icon(graphics={
        Ellipse(extent={{-70,-70},{70,70}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8),
        Text(extent={{-53,-17},{53,-53}}, textString="v", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableSpeedSensor;
