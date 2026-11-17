within MSLFaults.Electrical.Analog.Sources;
model FaultableCosineCurrent "Cosinusoidal current source with output faults"
  type FaultMode=enumeration(Normal "Nominal behaviour", OutputLoss "Attenuate the output towards zero", AmplitudeDrift "Scale the complete nominal waveform", AmplitudeDrop "Scale the complete nominal waveform", OffsetDrift "Additive output offset", FrequencyDrift "Scale the time-based waveform phase", StuckOutput "Interpolate towards a prescribed output");
  parameter Modelica.Units.SI.Current I(start=1) "Amplitude of cosine wave";
  parameter Modelica.Units.SI.Angle phase=0 "Phase of cosine wave";
  parameter Modelica.Units.SI.Frequency f(start=1) "Frequency of cosine wave";
  parameter Modelica.Units.SI.Current offset=0 "Output offset";
  parameter Modelica.Units.SI.Time startTime=0 "Time offset";

  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  extends Modelica.Electrical.Analog.Icons.CurrentSource;
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter Real amplitudeFault=0.5 "Scale factor applied to the complete nominal output at full effect";
  parameter Real offsetFault=0 "Additive electrical-output offset at full effect";
  parameter Real frequencyFault=0.5 "Frequency multiplier at full effect";
  parameter Real stuckOutput=0 "Prescribed electrical output at full effect";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Real nominalOutput "Waveform before output scaling, offset and stuck-value transformation";
  Real effectiveOutput "Electrical output after the selected fault transformation";

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
  effectiveOutput = if faultMode == FaultMode.OutputLoss then nominalOutput*(1-faultActivation)
    elseif faultMode == FaultMode.AmplitudeDrift or faultMode == FaultMode.AmplitudeDrop then nominalOutput*(1+faultActivation*(amplitudeFault-1))
    elseif faultMode == FaultMode.OffsetDrift then nominalOutput+faultActivation*offsetFault
    elseif faultMode == FaultMode.StuckOutput then nominalOutput+faultActivation*(stuckOutput-nominalOutput)
    else nominalOutput;
  nominalOutput = offset + (if time < startTime then 0 else I*cos(2*Modelica.Constants.pi*f*(1+faultActivation*(if faultMode==FaultMode.FrequencyDrift then frequencyFault-1 else 0))*(time-startTime)+phase));
  i = effectiveOutput;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{-71,70},{-68.4,69.8},{-63.5,67},{-58.6,61},{-53.6,52},{-48,38.6},{-40.98,18.6},{-26.21,-26.9},{-19.9,-44},{-14.2,-56.2},{-9.3,-64},{-4.4,-68.6},{0.5,-70},{5.5,-67.9},{10.4,-62.5},{15.3,-54.1},{20.9,-41.3},{28,-21.7},{35,0}}, color={128,128,128}, smooth=Smooth.Bezier, thickness=0.65),
        Line(points={{35,0},{44.8,29.9},{51.2,46.5},{56.8,58.1},{61.7,65.2},{66.7,69.2},{71.6,69.8}}, color={128,128,128}, thickness=0.65),
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>The nominal output uses a cosine wave after startTime and offset before it. OutputLoss scales the output towards zero; amplitude and offset modes change its magnitude; StuckOutput interpolates towards stuckOutput.</p><p>Set I, f, phase, offset and startTime. Connect p and n with the prescribed current sign convention and inspect effectiveOutput.</p><p>AmplitudeDrift and AmplitudeDrop use the same scale interpolation, including any nominal offset. StuckOutput uses a fixed target. FrequencyDrift scales the time-based phase expression directly; phase is not integrated from instantaneous frequency.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Sources.CosineCurrent</code>.</p></html>"));

  
end FaultableCosineCurrent;
