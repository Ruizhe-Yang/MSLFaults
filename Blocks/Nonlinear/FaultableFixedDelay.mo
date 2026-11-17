within MSLFaults.Blocks.Nonlinear;
block FaultableFixedDelay "Causal signal delay with variable delay and output faults"
  extends Modelica.Blocks.Interfaces.SISO;
  parameter Modelica.Units.SI.Time delayTime(start=1) "Delay time";
  type FaultMode=enumeration(Normal "Nominal component relation", DelayDrift "Interpolate towards the delay target", DelayIncrease "Interpolate towards the selected delay target", DelayDecrease "Interpolate towards the selected delay target", Bias "Add the configured output offset", Dropout "Attenuate the output towards zero");
  parameter FaultMode faultMode=FaultMode.Normal "Selected component-specific fault mechanism";
  parameter Real severity(min=0,max=1)=1 "Maximum activation-envelope level";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the internal fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the internal fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Time for a full unit-level transition";
  parameter Modelica.Units.SI.Time delayTimeFault(min=0)=2*delayTime "Delay target at full effect";
  parameter Real biasFault=1 "Additive output bias at full effect";
  final parameter Modelica.Units.SI.Time delayMax=max(delayTime,delayTimeFault) "Maximum retained delay history";
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1),y_raw;
  Modelica.Units.SI.Time delayTime_effective;

public
  parameter Boolean enabled=true "Enable requests for the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use the external trigger instead of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
     "External fault request when enabled by useExternalTrigger" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault state before transition shaping";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of activation starts, including initial activation";

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
  delayTime_effective=if faultMode==FaultMode.DelayDrift or faultMode==FaultMode.DelayIncrease or faultMode==FaultMode.DelayDecrease then delayTime+faultActivation*(delayTimeFault-delayTime) else delayTime;
  y_raw=delay(u,delayTime_effective,delayMax);
  y=if faultMode==FaultMode.Bias then y_raw+faultActivation*biasFault elseif faultMode==FaultMode.Dropout then (1-faultActivation)*y_raw else y_raw;


  annotation(
    Documentation(info="<html><p>Delay <code>u</code> using the Modelica <code>delay</code> operator. <code>DelayDrift</code>, <code>DelayIncrease</code> and <code>DelayDecrease</code> use the same interpolation from <code>delayTime</code> to <code>delayTimeFault</code>; choose the target to obtain the required direction of change.</p><p>The retained history is bounded by <code>delayMax=max(delayTime,delayTimeFault)</code>. Configure nonnegative delays. <code>Bias</code> adds <code>effectLevel*biasFault</code> to the delayed value and <code>Dropout</code> attenuates it towards zero. The delay operator supplies the prehistory and recovery follows the activation envelope.</p></html>"),
    Icon(graphics={
        Line(points={{-74,58},{-74,-64},{68,-64}}, color={128,128,128}, thickness=0.45, pattern=LinePattern.Solid),
        Line(points={{-63,-45},{-38,-45},{-38,40},{62,40}}, color={128,128,128}, thickness=0.5, pattern=LinePattern.Dash),
        Line(points={{-63,-45},{12,-45},{12,40},{62,40}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{-38,-5},{12,-5}}, color={64,64,64}, thickness=0.6, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-34,-34},{8,-12}}, textString="T", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableFixedDelay;
