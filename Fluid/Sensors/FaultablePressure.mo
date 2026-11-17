within MSLFaults.Fluid.Sensors;
model FaultablePressure "Pressure measurement with output and capture faults"
  extends Modelica.Fluid.Sensors.BaseClasses.PartialAbsoluteSensor(redeclare replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium);
  Modelica.Blocks.Interfaces.RealOutput p(final quantity="Pressure",final unit="Pa",displayUnit="bar",min=0) annotation(Placement(transformation(extent={{100,-10},{120,10}})));
  type FaultMode=enumeration(Normal "Nominal behaviour", Bias "Additive measurement bias", GainError "Measurement gain error", NoiseIncrease "Sinusoidal measurement disturbance", Stuck "Blend towards a prescribed output", Dropout "Blend towards zero output", Saturation "Blend towards symmetric output limits", Freeze "Hold the value captured on activation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  parameter Real biasFault=1 "Full-effect additive bias in the measured signal unit" ;
  parameter Real gainFault=0.9 "Full-effect gain factor" ;
  parameter Real noiseAmplitude=0.01 "Full-effect sinusoidal amplitude in the measured signal unit" ;
  parameter Modelica.Units.SI.Frequency noiseFrequency=37 "Frequency of the sinusoidal disturbance" ;
  parameter Real stuckValue=0 "Prescribed stuck output in the measured signal unit" ;
  parameter Real saturationLimit=1e12 "Symmetric output limit in the measured signal unit" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  Real measuredActual "Physical measurement before the output fault" ;

public
  parameter Boolean enabled=true "Enable the selected fault mechanism";
  parameter Boolean useExternalTrigger=false "Use trigger to request activation instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
     "External activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested fault activation";
  output Real effectLevel(min=0,max=1) "Realised severity-scaled fault envelope";
  output Integer activationCount "Number of rising fault activations";
  discrete Real captured(start=0,fixed=true) "Measurement captured at initialisation or activation" ;
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
  measuredActual=port.p;
  p=if faultMode==FaultMode.Freeze then measuredActual+faultActivation*(captured-measuredActual) else if faultMode==FaultMode.Bias then measuredActual+faultActivation*biasFault elseif faultMode==FaultMode.GainError then measuredActual*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.NoiseIncrease then measuredActual+faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time) elseif faultMode==FaultMode.Stuck then measuredActual+faultActivation*(stuckValue-measuredActual) elseif faultMode==FaultMode.Dropout then measuredActual*(1-faultActivation) elseif faultMode==FaultMode.Saturation then measuredActual+faultActivation*(min(saturationLimit,max(-saturationLimit,measuredActual))-measuredActual) else measuredActual;

  when {initial(),active} then captured=measuredActual; end when;
  annotation(
    Documentation(info="<html><p>Connect the inherited <code>port</code> to the fluid node. The physical measurement is <code>port.p</code>. <code>Freeze</code> blends towards the pressure captured at initialisation or a rising activation.</p><p><code>Bias</code> adds <code>biasFault</code>; <code>GainError</code> blends towards <code>gainFault</code>; <code>NoiseIncrease</code> adds a deterministic sinusoid. <code>Stuck</code> blends towards <code>stuckValue</code>, <code>Dropout</code> towards zero and <code>Saturation</code> towards symmetric clipping at <code>saturationLimit</code>. Magnitudes use the measured signal unit. These operations affect the reported signal only.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Fluid.Sensors.Pressure</code>.</p></html>"),
    Icon(graphics={
        Ellipse(extent={{-70,-70},{70,70}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8),
        Text(extent={{-53,-17},{53,-53}}, textString="Pa", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultablePressure;
