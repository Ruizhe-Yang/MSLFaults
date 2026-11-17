within MSLFaults.Blocks.Math;
block FaultableGain "Gain with drift, bias, capture and output limitations"
  parameter Real k(start=1,unit="1") "Gain value multiplied with input signal";
  Modelica.Blocks.Interfaces.RealInput u  "Input signal" annotation(Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput y  "Gain output after the selected fault effect" annotation(Placement(transformation(extent={{100,-10},{120,10}})));
  type FaultMode=enumeration(Normal "Nominal component relation", GainDrift "Interpolate the gain towards its target", Bias "Add the configured output offset", Stuck "Blend towards the prescribed fixed output", Dropout "Attenuate the output towards zero", Saturation "Blend towards the symmetrically limited output", Freeze "Blend towards the output captured at activation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected component-specific fault mechanism";
  parameter Real severity(min=0,max=1)=1 "Maximum activation-envelope level";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the internal fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the internal fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Time for a full unit-level transition";
  parameter Real kFault=0.5*k "Gain target at full effect";
  parameter Real biasFault=1 "Additive output bias at full effect";
  parameter Real stuckValue=0 "Prescribed output at full fixed-value effect";
  parameter Real saturationLimit=1e9 "Nonnegative symmetric output limit";
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1),k_effective,y_raw;

public
  parameter Boolean enabled=true "Enable requests for the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use the external trigger instead of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
     "External fault request when enabled by useExternalTrigger" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault state before transition shaping";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of activation starts, including initial activation";
  discrete Real captured(start=0,fixed=true) "Signal value captured at activation";
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  assert(saturationLimit>=0,"The gain output limit must be nonnegative.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  k_effective=if faultMode==FaultMode.GainDrift then k+faultActivation*(kFault-k) else k;
  y_raw=k_effective*u;
  y=if faultMode==FaultMode.Freeze then y_raw+faultActivation*(captured-y_raw) else if faultMode==FaultMode.Bias then y_raw+faultActivation*biasFault elseif faultMode==FaultMode.Stuck then y_raw+faultActivation*(stuckValue-y_raw) elseif faultMode==FaultMode.Dropout then (1-faultActivation)*y_raw elseif faultMode==FaultMode.Saturation then y_raw+faultActivation*(min(saturationLimit,max(-saturationLimit,y_raw))-y_raw) else y_raw;

  when {initial(),active} then captured=y_raw; end when;
  annotation(
    Documentation(info="<html><p>Multiply <code>u</code> by <code>k</code>. <code>GainDrift</code> interpolates the gain towards <code>kFault</code>; <code>Bias</code>, <code>Stuck</code>, <code>Dropout</code> and <code>Saturation</code> blend the nominal output towards their configured targets using <code>effectLevel</code>.</p><p><code>Freeze</code> captures <code>y_raw</code> at each activation and blends towards that value. <code>Stuck</code> instead uses <code>stuckValue</code>. Set a nonnegative <code>saturationLimit</code> for symmetric limiting. Recovery restores the current gain output after the envelope returns to zero.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{-100,-100},{-100,100},{100,0},{-100,-100}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,-140},{150,-100}}, textString="k=%k"),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-64,30},{25,-30}}, textString="k", textColor={255,0,0}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableGain;
